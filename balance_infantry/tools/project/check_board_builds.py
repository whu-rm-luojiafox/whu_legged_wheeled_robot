"""Audit configured compile databases for board isolation and shared ABI guards.

Run after configuring/building the chassis and gimbal presets. This checks build
selection, not control performance or hardware. Uses only the Python stdlib.
"""
import argparse
import json
from pathlib import Path


def check(root, directory, board):
    database = json.loads((directory / "compile_commands.json").read_text(encoding="utf-8"))
    assert database, f"Empty compile database: {directory}"
    files = set()
    for entry in database:
        source = Path(entry["file"]).resolve()
        assert source.is_relative_to(root), f"External source dependency: {source}"
        relative = source.relative_to(root).as_posix()
        files.add(relative)
        command = entry.get("command", " ".join(entry.get("arguments", [])))
        selected = "ROBOT_BOARD_" + board.upper() + "=1"
        opposite = "ROBOT_BOARD_" + ("GIMBAL" if board == "chassis" else "CHASSIS")
        assert selected in command and opposite not in command, f"Board macro mismatch: {relative}"
        other = "gimbal" if board == "chassis" else "chassis"
        assert f"src/{other}/" not in command, f"Other board source/include leaked: {relative}"
        assert f"platform/stm32f407/{other}/" not in command, f"Other board platform leaked: {relative}"
    assert "platform/stm32f407/robot_link_layout_checks.c" in files
    assert "src/common/components/controller/pid.c" in files
    assert "src/common/application/led_flow_task.c" in files
    assert "third_party/freertos/Source/tasks.c" in files
    entry_point = ("platform/stm32f407/chassis/src/main.c" if board == "chassis"
                   else "platform/stm32f407/gimbal/src/main.c")
    assert entry_point in files
    output = directory / f"{board}_firmware.elf"
    assert output.is_file(), f"Missing built firmware: {output}"
    print(f"{board}: {len(files)} translation units, isolated board inputs, {output.name}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--chassis-dir", type=Path, default=Path("build/chassis"))
    parser.add_argument("--gimbal-dir", type=Path, default=Path("build/gimbal"))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    for board, path in [("chassis", args.chassis_dir), ("gimbal", args.gimbal_dir)]:
        check(root, (root/path).resolve(), board)


if __name__ == "__main__":
    main()
