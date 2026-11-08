"""Resolve robot board roles by USB serial number, independent of tty numbering."""

import json
from pathlib import Path

from serial.tools import list_ports

DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "serial_ports.json"
ROLE_LABELS = {"port1": "左臂与云台", "port2": "右臂", "port3": "底盘"}
EXPECTED_IDS = {"port1": set(range(1, 9)), "port2": set(range(1, 7)), "port3": {7, 8, 9}}


def connected_boards():
    boards = {}
    for port in list_ports.comports():
        if (port.vid, port.pid) != (0x1A86, 0x55D3) or not port.serial_number:
            continue
        if port.serial_number in boards:
            raise ValueError(f"重复 USB 序列号：{port.serial_number}")
        boards[port.serial_number] = port.device
    return boards


def resolve_ports(config_path, overrides):
    config = json.loads(Path(config_path).read_text()) if Path(config_path).exists() else {"roles": {}}
    boards = connected_boards()
    resolved = {}
    for role, label in ROLE_LABELS.items():
        if overrides.get(role):
            resolved[role] = overrides[role]
            continue
        serial = config["roles"].get(role)
        if not serial:
            raise ValueError(f"{role}（{label}）尚未确认，请先运行 scripts/probe_serial.py")
        if serial not in boards:
            raise ValueError(f"{role}（{label}）控制板 {serial} 未连接")
        resolved[role] = boards[serial]
    devices = [str(Path(port).resolve()) for port in resolved.values()]
    if len(set(devices)) != 3:
        raise ValueError("三个角色必须对应不同的串口")
    return resolved
