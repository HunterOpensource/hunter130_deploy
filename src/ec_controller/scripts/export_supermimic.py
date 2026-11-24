#!/usr/bin/env python3
"""Export a 600/609-goal Hunter130 actor using the training Python environment.

Saved motor_parameters are checked against the training assets before compiling
PD gains and action scales. Legacy checkpoints with robot_pd are also supported.
Default pose and limits come from the recorded training source/assets.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path

import numpy as np
import torch

from model_config import write_model_section


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def physical_metadata(checkpoint: dict, asset_root: Path | None) -> dict:
    import supermimic.robots as robots
    from mjlab.entity import Entity

    def compile_robot():
        result = robots.robot_cfg("hunter130", asset_root)
        if len(result) not in (2, 3):
            raise ValueError("Unsupported training robot_cfg return signature")
        cfg, scales = result[:2]
        entity = Entity(cfg)
        model = entity.spec.compile()
        limits = result[2] if len(result) == 3 else {
            name: model.joint(name).range for name in entity.joint_names
        }
        return entity, model, scales, limits

    saved_motors = checkpoint.get("motor_parameters")
    saved_pd = checkpoint.get("robot_pd")
    if saved_motors is not None:
        robots.validate_motor_parameters(saved_motors, robots.load_motor_parameters(asset_root))
        entity, model, scales, limits = compile_robot()
    else:
        if not isinstance(saved_pd, dict) or not saved_pd:
            raise ValueError("Checkpoint must save motor_parameters or robot_pd")
        previous_pd = robots.HUNTER130_PD
        try:
            robots.HUNTER130_PD = {name: tuple(values) for name, values in saved_pd.items()}
            entity, model, scales, limits = compile_robot()
        finally:
            robots.HUNTER130_PD = previous_pd
    names = list(entity.joint_names)
    if len(names) != 25 or model.nu != 25:
        raise ValueError("Expected the 25-joint Hunter130 training model")
    key = model.key("init_state").qpos
    defaults, lower, upper, kp, kd = [], [], [], [], []
    for name in names:
        joint = model.joint(name)
        defaults.append(float(key[joint.qposadr[0]]))
        lower.append(float(limits[name][0]))
        upper.append(float(limits[name][1]))
        actuator_ids = np.flatnonzero(model.actuator_trnid[:, 0] == joint.id)
        if len(actuator_ids) != 1:
            raise ValueError(f"Expected exactly one actuator for {name}")
        actuator = int(actuator_ids[0])
        gain = float(model.actuator_gainprm[actuator, 0])
        damping = -float(model.actuator_biasprm[actuator, 2])
        group = name.removeprefix("left_").removeprefix("right_").removesuffix("_joint")
        expected_pd = (
            (saved_motors[name]["stiffness"], saved_motors[name]["damping"])
            if saved_motors is not None else tuple(saved_pd[group])
        )
        if saved_motors is not None and scales[name] != saved_motors[name]["action_scale"]:
            raise ValueError(f"Action scale differs from checkpoint for {name}")
        if (gain, damping) != expected_pd:
            raise ValueError(f"Compiled PD differs from saved checkpoint PD for {name}")
        if float(model.actuator_biasprm[actuator, 1]) != -gain:
            raise ValueError(f"Unexpected actuator position bias for {name}")
        kp.append(gain)
        kd.append(damping)
    vectors = [defaults, lower, upper, kp, kd, [scales[n] for n in names]]
    if not np.isfinite(np.asarray(vectors)).all():
        raise ValueError("Non-finite training physical parameters")
    if (
        np.any(np.asarray(lower) >= upper)
        or np.any(np.asarray(defaults) < lower)
        or np.any(np.asarray(defaults) > upper)
    ):
        raise ValueError("Invalid training default pose or limits")
    if (
        np.any(np.asarray(kp) <= 0)
        or np.any(np.asarray(kd) < 0)
        or np.any(np.asarray(vectors[-1]) <= 0)
    ):
        raise ValueError("Invalid training PD gains or action scales")
    params = {
        "joint_names": names, "stand_position": defaults, "lower": lower, "upper": upper,
        "kp": kp, "kd": kd, "action_scale": [float(scales[n]) for n in names],
        "stand_height": float(key[2]),
    }
    if saved_motors is not None:
        params["motor_parameters"] = saved_motors
    return params


def verify(policy, model_path: Path, samples: int, goal_size: int) -> dict:
    try:
        import onnxruntime as ort
    except ImportError as error:
        raise RuntimeError(
            "Install onnxruntime in the export environment for numerical validation"
        ) from error
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    session = ort.InferenceSession(
        str(model_path), options, providers=["CPUExecutionProvider"]
    )
    rng = np.random.default_rng(130)
    largest = 0.0
    with torch.inference_mode():
        for _ in range(samples):
            obs = rng.normal(size=(1, 810)).astype(np.float32)
            goal = rng.normal(size=(1, goal_size)).astype(np.float32)
            expected = policy(torch.from_numpy(obs), torch.from_numpy(goal)).numpy()
            actual = session.run(["action"], {"policy": obs, "goal": goal})[0]
            np.testing.assert_allclose(actual, expected, atol=2e-5, rtol=2e-4)
            largest = max(largest, float(np.max(np.abs(actual - expected))))
    return {
        "reference": "checkpoint_pytorch_policy",
        "runtime": ort.__version__,
        "samples": samples,
        "max_absolute_error": largest,
        "atol": 2e-5,
        "rtol": 2e-4,
    }


def export_reference(source: Path, destination: Path, metadata: dict) -> dict:
    """Export unscaled 60-feature reference rows from an optional training NPZ."""
    from supermimic.envs.mdp.motion_library import RefState, goal_features

    with np.load(source) as data:
        if list(data["joint_names"]) != metadata["joint_names"] or not np.isclose(
            float(data["fps"]), 50
        ):
            raise ValueError("Reference must use the training joint order at 50 Hz")
        pelvis = list(data["body_names"]).index("pelvis")
        ref = RefState(
            root_pos_w=torch.from_numpy(
                data["body_pos_w"][:, pelvis].astype(np.float32)
            ),
            root_quat_w=torch.from_numpy(
                data["body_quat_w"][:, pelvis].astype(np.float32)
            ),
            joint_pos=torch.from_numpy(data["joint_pos"].astype(np.float32)),
            root_lin_vel_w=torch.from_numpy(
                data["body_lin_vel_w"][:, pelvis].astype(np.float32)
            ),
            root_ang_vel_w=torch.from_numpy(
                data["body_ang_vel_w"][:, pelvis].astype(np.float32)
            ),
            joint_vel=torch.from_numpy(data["joint_vel"].astype(np.float32)),
        )
        features = goal_features(
            ref, torch.tensor(metadata["stand_position"], dtype=torch.float32)
        ).numpy()
    if (
        features.ndim != 2
        or features.shape[1] != 60
        or len(features) < 2
        or not np.isfinite(features).all()
    ):
        raise ValueError("Invalid training reference features")
    destination.mkdir(parents=True, exist_ok=True)
    path = destination / (source.stem + ".csv")
    np.savetxt(
        path,
        np.column_stack([np.arange(len(features)) * 0.02, features]),
        delimiter=",",
        header=",".join(["time"] + [f"goal_{i}" for i in range(60)]),
        comments="",
        fmt="%.9g",
    )
    record = {
        "source": str(source.resolve()),
        "source_sha256": sha256(source),
        "path": path.name,
        "sha256": sha256(path),
        "frames": len(features),
    }
    (destination / (source.stem + ".source.json")).write_text(
        json.dumps(record, indent=2) + "\n"
    )
    return record


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--supermimic-root", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "config",
    )
    parser.add_argument("--asset-root", type=Path)
    parser.add_argument("--model-name",
                        help="Basename for the exported ONNX model")
    parser.add_argument("--joint-target-clip", choices=("joint_limits", "none"),
                        default="joint_limits",
                        help="Target clipping verified in the training action implementation")
    parser.add_argument(
        "--motion",
        type=Path,
        help="Optional training NPZ to export as 61-column reference features",
    )
    parser.add_argument("--verify-samples", type=int, default=64)
    args = parser.parse_args()
    if args.verify_samples < 1:
        parser.error("--verify-samples must be positive")
    if args.model_name is not None and not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", args.model_name):
        parser.error("--model-name must be a filename basename without path separators")
    root = args.supermimic_root.resolve()
    sys.path.insert(0, str(root / "src"))
    from supermimic.policy import nets

    Policy = nets.Policy
    checkpoint_policy_cfg = getattr(nets, "checkpoint_policy_cfg", lambda values: nets.PolicyCfg(**values))

    torch.set_num_threads(1)
    checkpoint_path = args.checkpoint.resolve()
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    config = checkpoint["config"]
    policy_cfg = checkpoint_policy_cfg(config["policy"])
    anchor_obs = config["env"].get("anchor_obs", False)
    if config["env"].get("robot") != "hunter130" or not isinstance(anchor_obs, bool):
        raise ValueError("Expected a Hunter130 checkpoint with a boolean anchor_obs")
    goal_size = 609 if anchor_obs else 600
    if config.get("policy_trainer", {}).get("anchor_obs", anchor_obs) != anchor_obs:
        raise ValueError("Environment and policy trainer anchor settings disagree")
    if (
        policy_cfg.num_joints != 25
        or policy_cfg.history != 10
        or policy_cfg.goal_dim != goal_size
        or config["env"]["future_frames"] != 10
    ):
        raise ValueError(f"Expected the 810+{goal_size} -> 25 Hunter130 actor contract")
    dt = float(config["world_model"]["state_spec"]["dt"])
    if not np.isfinite(dt) or abs(dt - 0.02) > 1e-9:
        raise ValueError("Expected a 50 Hz training control period")
    policy = Policy(policy_cfg).eval()
    # Strict loading also verifies that the saved observation/goal scale buffers
    # and every layer shape belong to this network definition.
    policy.load_state_dict(checkpoint["policy"], strict=True)
    asset_root = args.asset_root or config["env"].get("asset_root")
    # Reject mismatched motor assets before replacing any deployment output.
    physics = physical_metadata(checkpoint, asset_root)
    output = args.output_dir.resolve()
    directory = output / "policy"
    directory.mkdir(parents=True, exist_ok=True)
    (output / "references").mkdir(parents=True, exist_ok=True)
    model_path = directory / f"{args.model_name or 'mimic'}.onnx"
    torch.onnx.export(
        policy,
        (torch.zeros(1, 810), torch.zeros(1, goal_size)),
        str(model_path),
        input_names=["policy", "goal"],
        output_names=["action"],
        opset_version=18,
        dynamo=False,
    )
    import onnx

    onnx.checker.check_model(str(model_path))
    metadata = {
        "schema_version": 2,
        "model_type": "supermimic",
        "anchor_obs": anchor_obs,
        "position_feedback": "zero" if anchor_obs else "none",
        "action_mode": "default_offset",
        "joint_target_clip": args.joint_target_clip,
        "input_size": 810,
        "goal_size": goal_size,
        "joint_count": 25,
        "reference_frames": 10,
        "reference_width": 60,
        "history_frames": 10,
        "proprio_width": 81,
        "goal_start_offset": 0,
        "reference_joint_position": "relative_to_default",
        "history_layout": "term_major_oldest_first",
        "goal_layout": "10x60_then_zero_position3_then_torso_rotation_first_two_columns_row_major6" if anchor_obs else "future_frame_major",
        "normalization": "embedded_in_onnx",
        "policy_dt": dt,
        "obs_term_scale": list(policy_cfg.obs_term_scale),
        "goal_term_scale": list(policy_cfg.goal_term_scale),
    }
    if anchor_obs:
        metadata.update({
            "torso_joint_names": ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"],
            "torso_joint_axes": [0., 0., 1., 1., 0., 0., 0., 1., 0.],
        })
    metadata.update(physics)
    verification = verify(policy, model_path, args.verify_samples, goal_size)
    if args.motion:
        export_reference(args.motion, output / "references", metadata)
    config_file = directory / "mimic.yaml"
    write_model_section(config_file, "mimic_model", metadata)
    print(json.dumps({
        "model": str(model_path), "configuration": str(config_file),
        "iteration": int(checkpoint["iteration"]), "verification": verification,
    }, indent=2))


if __name__ == "__main__":
    main()
