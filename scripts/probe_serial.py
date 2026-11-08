#!/usr/bin/env python3
"""只读识别三块 USB 控制板及舵机 ID，不改变扭矩、校准或目标位置。"""

import argparse
import json
from pathlib import Path

from serial_config import DEFAULT_CONFIG, EXPECTED_IDS, ROLE_LABELS, connected_boards


def probe(port, baudrate):
    from lerobot.motors import Motor, MotorNormMode, SerialMotorsBus

    bus = SerialMotorsBus(
        port=port,
        motors={f"motor_{i}": Motor(i, "sts3215", MotorNormMode.DEGREES) for i in range(1, 10)},
    )
    bus.default_baudrate = baudrate
    found = {}
    try:
        bus.connect(handshake=False)
        for motor_id in range(1, 10):
            model = bus.ping(motor_id, num_retry=1)
            if model is not None:
                found[motor_id] = model
    finally:
        # The default disconnect disables torque; identification must not do that.
        bus.disconnect(disable_torque=False)
    return found


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--list", action="store_true", help="仅列出 USB 板，不打开串口")
    parser.add_argument("--assign", nargs=3, metavar="portN=SERIAL", help="确认后保存三个角色的序列号")
    args = parser.parse_args()
    if args.baudrate <= 0:
        parser.error("baudrate 必须为正数")
    try:
        config = json.loads(args.config.read_text())
        boards = connected_boards()
    except (OSError, ValueError) as exc:
        parser.exit(1, f"{exc}\n")
    if args.assign:
        try:
            roles = dict(item.split("=", 1) for item in args.assign)
        except ValueError:
            parser.error("格式：--assign port1=序列号 port2=序列号 port3=序列号")
        if set(roles) != set(ROLE_LABELS) or len(set(roles.values())) != 3:
            parser.error("必须为 port1、port2、port3 分别指定不同的序列号")
        if any(serial not in config["boards"] or serial not in boards for serial in roles.values()):
            parser.error("只能分配已记录且当前在线的三块控制板")
        config["roles"] = roles
        args.config.write_text(json.dumps(config, indent=2, ensure_ascii=False) + "\n")
        print(f"已保存角色映射：{args.config}")
        return
    failed = False
    for serial in config["boards"]:
        port = boards.get(serial)
        if not port:
            print(f"{serial}: 未连接")
            failed = True
            continue
        print(f"{serial}: {port}", flush=True)
        if args.list:
            continue
        try:
            found = probe(port, args.baudrate)
        except Exception as exc:
            print(f"  打开或读取失败：{exc}")
            failed = True
            continue
        print(f"  响应 ID/型号：{found or '无响应（请检查舵机供电、总线连接和波特率）'}")
        candidates = [f"{role}（{ROLE_LABELS[role]}）" for role, ids in EXPECTED_IDS.items() if set(found) == ids]
        print(f"  候选角色：{', '.join(candidates) if candidates else '暂时无法判断'}")
        print("  候选仅按项目预期 ID 1–9 判断；缺失响应可能误判，需核对实际接线。")
        if not found:
            failed = True
    if failed:
        parser.exit(1, "部分设备未连接或无响应；未更改角色映射。\n")


if __name__ == "__main__":
    main()
