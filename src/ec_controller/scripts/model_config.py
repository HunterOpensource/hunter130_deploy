"""Update model parameters in the corresponding ROS policy configuration."""

import fcntl
import os
import tempfile
from pathlib import Path

import yaml


def write_model_section(path: Path, section: str, parameters: dict) -> None:
    # Keep the other controllers, comments and launch substitutions verbatim.
    template = Path(__file__).resolve().parents[1] / "config/policy" / path.name
    directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        fcntl.flock(directory, fcntl.LOCK_EX)
        original = path if path.exists() else template
        text = original.read_text()
        lines = text.splitlines(keepends=True)
        root = yaml.compose(text)
        for key, value in root.value:
            if key.value == section:
                start = key.start_mark.line
                end = value.end_mark.line + (value.end_mark.column > 0)
                while end > start and (not lines[end - 1].strip() or
                                       lines[end - 1].lstrip().startswith("#")):
                    end -= 1
                break
        else:
            raise ValueError(f"Missing {section} in {original}")
        block = yaml.safe_dump({section: {"ros__parameters": parameters}},
                               sort_keys=False, default_flow_style=None, width=110)
        updated = "".join(lines[:start]) + block + "".join(lines[end:])
        expected = yaml.safe_load(text)
        expected[section] = {"ros__parameters": parameters}
        if yaml.safe_load(updated) != expected:
            raise ValueError(f"Unable to update {section} in {original}")
        with tempfile.NamedTemporaryFile(mode="w", dir=path.parent, delete=False) as out:
            temporary = Path(out.name)
            try:
                out.write(updated)
                os.fchmod(out.fileno(), original.stat().st_mode & 0o777)
            except BaseException:
                temporary.unlink()
                raise
        try:
            temporary.replace(path)
        finally:
            temporary.unlink(missing_ok=True)
    finally:
        os.close(directory)
