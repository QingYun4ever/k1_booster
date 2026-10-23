#!/usr/bin/env python3
"""Run ROS-free behavioral regressions and validate shipped XML/YAML contracts."""
import argparse
import ast
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", default=os.environ.get("CXX", "g++"))
    args = parser.parse_args()
    compiler = shutil.which(args.cxx)
    if not compiler:
        raise SystemExit(f"C++ compiler unavailable: {args.cxx}")
    suites = [
        ("referee", ["src/game_controller/test/test_game_controller_packet.cpp"], ["src/game_controller/include"]),
        ("sharing", ["src/k1_teammate_ball/test/test_teammate_ball_sharing.cpp", "src/k1_teammate_ball/src/teammate_ball_sharing.cpp"], ["src/k1_teammate_ball/include"]),
        ("match", ["src/brain/test/match_policy_test.cpp"], ["src/brain/include"]),
        ("team_wire", ["src/brain/test/team_wire_test.cpp", "src/k1_teammate_ball/src/teammate_ball_sharing.cpp"], ["src/brain/include", "src/k1_teammate_ball/include"]),
        ("motion", ["src/brain/test/motion_policy_test.cpp"], ["src/brain/include"]),
        ("timing", ["src/brain/test/tactical_timing_test.cpp"], ["src/brain/include"]),
    ]
    with tempfile.TemporaryDirectory(prefix="qybot-native-") as directory:
        for name, sources, includes in suites:
            executable = Path(directory) / (name + (".exe" if os.name == "nt" else ""))
            command = [compiler, "-std=c++17", "-Wall", "-Wextra", "-Wno-unknown-pragmas", "-O0"]
            command.extend("-I" + str(ROOT / include) for include in includes)
            command.extend(str(ROOT / source) for source in sources)
            command.extend(["-o", str(executable)])
            subprocess.run(command, check=True, cwd=ROOT)
            subprocess.run([str(executable)], check=True, cwd=ROOT)
    xml_files = list((ROOT / "src/brain/behavior_trees").rglob("*.xml"))
    definitions = {}
    parsed = {}
    for path in xml_files:
        try:
            root = ET.parse(path).getroot()
        except ET.ParseError as error:
            raise SystemExit(f"Malformed BT XML {path}: {error}") from error
        parsed[path.resolve()] = root
        for tree in root.findall("BehaviorTree"):
            definitions.setdefault(tree.attrib["ID"], []).append(path)
        for include in root.findall("include"):
            target = (path.parent / include.attrib["path"]).resolve()
            if not target.is_file():
                raise SystemExit(f"Missing BT include: {path}: {target}")
    for path, root in parsed.items():
        for subtree in root.iter("SubTree"):
            if subtree.attrib["ID"] not in definitions:
                raise SystemExit(f"Undefined BT subtree {subtree.attrib['ID']} in {path}")
    print(f"XML: {len(xml_files)} behavior trees parsed; includes/subtrees resolved")
    for path in [ROOT / "configure_demo.py", ROOT / "src/brain/launch/launch.py", ROOT / "src/game_controller/launch/launch.py"]:
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    print("Python: modified configuration and launch modules parsed")
    try:
        import yaml
    except ImportError:
        print("YAML validation unavailable: install PyYAML", flush=True)
        return 1
    config = yaml.safe_load((ROOT / "src/brain/config/config.yaml").read_text(encoding="utf-8"))["brain_node"]["ros__parameters"]
    game = config["game"]
    assert 1 <= game["player_id"] <= game["number_of_players"] <= 11
    assert 1 <= game["goalkeeper_id"] <= game["number_of_players"]
    assert game["controller_timeout_ms"] > 0
    print("YAML: player/goalkeeper roster and controller timeout validated")
    print("Native checks passed. ROS/DDS/Jetson/robot runtime is not exercised by this command.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
