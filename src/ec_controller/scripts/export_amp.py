#!/usr/bin/env python3
"""Export Hunter130 AMP from a trusted checkpoint and its saved run parameters.

Run with the training Python environment plus onnx and onnxruntime installed.
No simulator, viewer, GPU, or hardware connection is needed.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import onnx
import onnxruntime as ort
import torch
import yaml

from model_config import write_model_section


def read_params(path: Path) -> dict:
    # Training YAML contains Python tags. BaseLoader reads data without
    # constructing the tagged objects or importing their functions.
    return yaml.load(path.read_text(), Loader=yaml.BaseLoader)


def physical_metadata(root: Path, env: dict, agent: dict) -> dict:
    actor = env["observations"]["actor"]
    terms = ["base_ang_vel", "projected_gravity", "command", "joint_pos", "joint_vel", "actions"]
    if list(actor["terms"]) != terms or actor["history_ordering"] != "time":
        raise ValueError("WalkController requires the six raw terms in time-major order")
    if actor["history_length"] != "4" or actor["concatenate_terms"] != "true":
        raise ValueError("Expected four concatenated history frames")
    for term in actor["terms"].values():
        if term["clip"] != "null" or term["scale"] != "null" or int(term["delay_max_lag"]):
            raise ValueError("WalkController requires unscaled, unclipped, undelayed observations")
    action = env["actions"]["joint_pos"]
    if (action["clip"] != "null" or agent["clip_actions"] != "null"
            or action["use_default_offset"] != "true"):
        raise ValueError("Expected unclipped joint-position actions with default offsets")
    xml = root / "src/assets/robots/hunter130/assets/hunter130.xml"
    xml_joints = ET.parse(xml).findall("./worldbody//joint")
    names = [joint.attrib["name"] for joint in xml_joints]
    robot = env["scene"]["entities"]["robot"]
    motors = {motor["target_names_expr"][0]: motor for motor in robot["articulation"]["actuators"]}
    if len(names) != 25 or list(motors) != names or set(action["scale"]) != set(names):
        raise ValueError("Saved actuators, action scales and XML must agree on all 25 joints")

    def default(name: str, field: str) -> float:
        matches = [float(v) for pattern, v in robot["init_state"][field].items()
                   if re.fullmatch(pattern, name)]
        if len(matches) > 1:
            raise ValueError(f"Ambiguous default {field} for {name}")
        return matches[0] if matches else 0.0

    joints = []
    for index, (name, joint) in enumerate(zip(names, xml_joints)):
        lower, upper = map(float, joint.attrib["range"].split())
        motor = motors[name]
        joints.append({
            "index": index, "name": name, "default_position": default(name, "joint_pos"),
            "kp": float(motor["stiffness"]), "kd": float(motor["damping"]),
            "action_scale": float(action["scale"][name]),
            "position_min": lower, "position_max": upper,
            "torque_limit": float(motor["effort_limit"]),
        })
    velocities = [default(name, "joint_vel") for name in names]
    if any(velocities):
        raise ValueError("WalkController requires zero default joint velocities")
    timestep = float(env["sim"]["mujoco"]["timestep"])
    period = timestep * int(env["decimation"])
    if not np.isclose(period, 0.02):
        raise ValueError("Expected a 50 Hz policy")
    return {
        "input_name": "obs", "input_shape": [1, 336],
        "output_name": "actions", "output_shape": [1, 25],
        "frame_size": 84, "history_length": 4, "history_layout": "time_major",
        "history_reset": "repeat_first_observation", "policy_frequency_hz": 50,
        "policy_period_s": period, "action_transform": "default_position + action_scale * action",
        "joint_names": names,
        "joints": {j["name"]: {k: v for k, v in j.items() if k not in ("index", "name")}
                   for j in joints},
    }


def verify(policy, normalizer, model: Path, samples: int) -> dict:
    session_options = ort.SessionOptions()
    session_options.intra_op_num_threads = 1
    session = ort.InferenceSession(str(model), sess_options=session_options,
                                   providers=["CPUExecutionProvider"])
    rng = np.random.default_rng(35000)
    max_error = 0.0
    with torch.inference_mode():
        for i in range(samples):
            if i == 0:
                frames = np.zeros((4, 84), dtype=np.float32)
                frames[:, 5] = -1.0
                obs = frames.reshape(1, 336)
            else:
                # Exercise the distribution learned by the saved normalizer,
                # including independent history frames and previous actions.
                obs = (normalizer.mean.numpy() + rng.standard_normal((1, 336)) *
                       normalizer.std.numpy()).astype(np.float32)
            expected = policy(torch.from_numpy(obs)).numpy()
            actual = session.run(["actions"], {"obs": obs})[0]
            if not np.isfinite(actual).all():
                raise ValueError("ONNX returned non-finite actions")
            np.testing.assert_allclose(actual, expected, rtol=1e-4, atol=2e-5)
            max_error = max(max_error, float(np.abs(actual - expected).max()))
    return {"samples": samples, "seed": 35000, "max_abs_action_error": max_error,
            "rtol": 1e-4, "atol": 2e-5, "onnxruntime_version": ort.__version__}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--training-root", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--verify-samples", type=int, default=256)
    args = parser.parse_args()
    if args.verify_samples < 1:
        parser.error("--verify-samples must be positive")
    root, checkpoint = args.training_root.resolve(), args.checkpoint.resolve()
    sys.path.insert(0, str(root))
    from rsl_rl.modules import ActorCritic, EmpiricalNormalization

    env_path = checkpoint.parent / "params/env.yaml"
    agent_path = checkpoint.parent / "params/agent.yaml"
    agent = read_params(agent_path)
    metadata = physical_metadata(root, read_params(env_path), agent)
    saved = torch.load(checkpoint, map_location="cpu", weights_only=False)
    state = saved["model_state_dict"]
    iteration = int(saved["iter"])
    if checkpoint.stem != f"model_{iteration}":
        raise ValueError("Checkpoint filename and saved iteration disagree")
    if state["actor.0.weight"].shape[1] != 336 or state["std"].shape != (25,):
        raise ValueError("Checkpoint does not implement the 336 -> 25 AMP contract")
    cfg = agent["policy"]
    actor_critic = ActorCritic(
        336, state["critic.0.weight"].shape[1], 25,
        actor_hidden_dims=list(map(int, cfg["actor_hidden_dims"])),
        critic_hidden_dims=list(map(int, cfg["critic_hidden_dims"])),
        activation=cfg["activation"], noise_std_type=cfg["noise_std_type"],
    )
    actor_critic.load_state_dict(state, strict=True)
    if agent["empirical_normalization"] != "true":
        raise ValueError("Expected a saved empirical observation normalizer")
    normalizer = EmpiricalNormalization(shape=[336])
    normalizer.load_state_dict(saved["obs_norm_state_dict"], strict=True)
    policy = torch.nn.Sequential(normalizer, actor_critic.actor).eval()
    torch.set_num_threads(1)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    model = args.output_dir / "amp.onnx"
    torch.onnx.export(
        policy, torch.zeros(1, 336), str(model), export_params=True, opset_version=18,
        input_names=["obs"], output_names=["actions"],
        dynamic_axes={"obs": {0: "batch"}, "actions": {0: "batch"}},
        external_data=False, dynamo=False,
    )
    onnx.checker.check_model(onnx.load(model), full_check=True)
    verification = verify(policy, normalizer, model, args.verify_samples)
    write_model_section(model.parent / "amp.yaml", "amp_model", metadata)
    print(json.dumps({"model": str(model), **verification}, indent=2))


if __name__ == "__main__":
    main()
