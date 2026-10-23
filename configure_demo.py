#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""K1 演示常用网络/球员配置的交互式快捷工具。"""

from __future__ import annotations

import re
import shutil
import sys
from datetime import datetime
from pathlib import Path


ROOT = Path(__file__).resolve().parent
BRAIN_CONFIG = ROOT / "src" / "brain" / "config" / "config.yaml"
WEBUI_CONFIG = ROOT / "src" / "k1_robot_webui_client" / "config" / "webui_client.yaml"
GC_LAUNCH = ROOT / "src" / "game_controller" / "launch" / "launch.py"


def print_usage() -> None:
    print("用法: python configure_demo.py")
    print()
    print("交互式快捷修改以下配置:")
    print("  src/brain/config/config.yaml: team_id, player_id, number_of_players, goalkeeper_id, game_control_ip")
    print("  src/k1_robot_webui_client/config/webui_client.yaml: server_base_url")
    print("  src/game_controller/launch/launch.py: ip_white_list")


def strip_quotes(value: str) -> str:
    value = value.strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in {'"', "'"}:
        return value[1:-1]
    return value


def normalize_server_url(value: str) -> str:
    value = value.strip()
    if not value:
        return value
    if value.startswith("http://") or value.startswith("https://"):
        return value.rstrip("/")
    return f"http://{value}:8000"


def read_file(path: Path) -> str:
    if not path.exists():
        raise FileNotFoundError(f"缺少文件: {path}")
    return path.read_text(encoding="utf-8")


def find_yaml_value(text: str, key: str, default: str = "") -> str:
    match = re.search(rf"^\s*{re.escape(key)}:\s*([^#\n]+)", text, re.MULTILINE)
    return strip_quotes(match.group(1)) if match else default


def find_server_url(text: str) -> str:
    return find_yaml_value(text, "server_base_url", "")


def find_launch_ip(text: str, default: str = "") -> str:
    match = re.search(r'"ip_white_list"\s*:\s*\[\s*\n\s*"([^"]+)"', text)
    return match.group(1) if match else default


def ask_value(prompt: str, default: str) -> str:
    suffix = f" [{default}]" if default else ""
    value = input(f"{prompt}{suffix}: ").strip()
    return value or default


def ask_int(prompt: str, default: str, minimum: int | None = None, maximum: int | None = None) -> str:
    while True:
        value = ask_value(prompt, default)
        try:
            parsed = int(value)
        except ValueError:
            print("请输入整数。")
            continue
        if minimum is not None and parsed < minimum:
            print(f"数值必须 >= {minimum}。")
            continue
        if maximum is not None and parsed > maximum:
            print(f"数值必须 <= {maximum}。")
            continue
        return str(parsed)


def replace_yaml_scalar(text: str, key: str, value: str, quoted: bool = False) -> str:
    formatted = f'"{value}"' if quoted else value
    pattern = re.compile(rf"^(\s*{re.escape(key)}:\s*)([^#\n]*)(\s*#.*)?$", re.MULTILINE)

    def repl(match: re.Match[str]) -> str:
        comment = match.group(3) or ""
        return f"{match.group(1)}{formatted}{comment}"

    new_text, count = pattern.subn(repl, text, count=1)
    if count != 1:
        raise ValueError(f"找不到 YAML 键: {key}")
    return new_text


def replace_launch_whitelist_ip(text: str, ip: str) -> str:
    pattern = re.compile(r'("ip_white_list"\s*:\s*\[\s*\n)(\s*)"[^"]*"(,?\s*\n\s*\])')

    def repl(match: re.Match[str]) -> str:
        return f'{match.group(1)}{match.group(2)}"{ip}"{match.group(3)}'

    new_text, count = pattern.subn(repl, text, count=1)
    if count != 1:
        raise ValueError("找不到 launch.py 中的 ip_white_list 条目")
    return new_text


def backup(path: Path, stamp: str) -> None:
    backup_path = path.with_name(f"{path.name}.bak.{stamp}")
    shutil.copy2(path, backup_path)
    print(f"已备份: {path.relative_to(ROOT)} -> {backup_path.relative_to(ROOT)}")


def write_if_changed(path: Path, old_text: str, new_text: str, stamp: str) -> None:
    if old_text == new_text:
        print(f"未变化: {path.relative_to(ROOT)}")
        return
    backup(path, stamp)
    path.write_text(new_text, encoding="utf-8", newline="")
    print(f"已更新: {path.relative_to(ROOT)}")


def main() -> int:
    if any(arg in {"-h", "--help"} for arg in sys.argv[1:]):
        print_usage()
        return 0

    brain_text = read_file(BRAIN_CONFIG)
    webui_text = read_file(WEBUI_CONFIG)
    launch_text = read_file(GC_LAUNCH)

    current_team_id = find_yaml_value(brain_text, "team_id", "66")
    current_player_id = find_yaml_value(brain_text, "player_id", "1")
    current_players = find_yaml_value(brain_text, "number_of_players", "3")
    current_goalkeeper = find_yaml_value(brain_text, "goalkeeper_id", "3")
    current_gc_ip = find_yaml_value(brain_text, "game_control_ip", find_launch_ip(launch_text, "192.168.74.2"))
    current_server = find_server_url(webui_text)

    print("K1 演示快速配置")
    print(f"工作目录: {ROOT}")
    print()

    team_id = ask_int("team_id", current_team_id, minimum=1, maximum=254)
    number_of_players = ask_int("number_of_players", current_players, minimum=1, maximum=11)
    player_id = ask_int("player_id", current_player_id, minimum=1, maximum=int(number_of_players))
    goalkeeper_id = ask_int("goalkeeper_id (全队一致)", current_goalkeeper, minimum=1, maximum=int(number_of_players))
    game_control_ip = ask_value("GameController IP", current_gc_ip)
    server_base_url = normalize_server_url(ask_value("Windows WebUI 后端 IP 或 URL", current_server))

    print()
    print("将应用以下配置:")
    print(f"  team_id: {team_id}")
    print(f"  player_id: {player_id}")
    print(f"  number_of_players: {number_of_players}")
    print(f"  goalkeeper_id: {goalkeeper_id}")
    print(f"  game_control_ip / ip_white_list: {game_control_ip}")
    print(f"  server_base_url: {server_base_url}")
    confirm = input("继续 [Y/n]: ").strip().lower()
    if confirm in {"n", "no"}:
        print("已取消")
        return 0

    new_brain = brain_text
    new_brain = replace_yaml_scalar(new_brain, "team_id", team_id)
    new_brain = replace_yaml_scalar(new_brain, "player_id", player_id)
    new_brain = replace_yaml_scalar(new_brain, "number_of_players", number_of_players)
    new_brain = replace_yaml_scalar(new_brain, "goalkeeper_id", goalkeeper_id)
    new_brain = replace_yaml_scalar(new_brain, "game_control_ip", game_control_ip, quoted=True)

    new_webui = replace_yaml_scalar(webui_text, "server_base_url", server_base_url, quoted=True)
    new_launch = replace_launch_whitelist_ip(launch_text, game_control_ip)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    write_if_changed(BRAIN_CONFIG, brain_text, new_brain, stamp)
    write_if_changed(WEBUI_CONFIG, webui_text, new_webui, stamp)
    write_if_changed(GC_LAUNCH, launch_text, new_launch, stamp)

    print()
    print("完成")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
