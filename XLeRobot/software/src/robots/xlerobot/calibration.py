"""Convert recorded STS3215 coordinates to writable single-turn limits."""

from dataclasses import replace

from lerobot.motors import MotorCalibration


def center_sts3215_calibration(
    calibration: dict[str, MotorCalibration],
) -> dict[str, MotorCalibration]:
    """Translate out-of-range limits and their homing offset together.

    STS3215 reports position = encoder - homing_offset. Translating both limits
    by delta therefore requires subtracting delta from the homing offset.
    Never clamp or wrap endpoints: either would change the measured joint span.
    Validate the whole bus before its caller writes any calibration registers.
    """
    centered = {}
    for name, cal in calibration.items():
        low, high = cal.range_min, cal.range_max
        if not 0 < high - low <= 4095:
            raise ValueError(
                f"{name}: recorded range [{low}, {high}] does not fit one turn. "
                "Restart calibration; move only through the joint's usable range without repeated turns."
            )
        # Limits require -low <= delta <= 4095-high. The sign-magnitude
        # homing register also requires offset-2047 <= delta <= offset+2047.
        # Centering is only a preference: it can exceed the offset register
        # even when a different translation fits both constraints.
        delta_min = max(-low, cal.homing_offset - 2047)
        delta_max = min(4095 - high, cal.homing_offset + 2047)
        if delta_min > delta_max:
            raise ValueError(
                f"{name}: range [{low}, {high}] and homing offset {cal.homing_offset} "
                "cannot both fit the STS3215 registers. "
                "Restart calibration with this joint at the middle of its usable range."
            )
        preferred_delta = 0 if 0 <= low < high <= 4095 else 2047 - (low + high) // 2
        delta = max(delta_min, min(delta_max, preferred_delta))
        offset = cal.homing_offset - delta
        centered[name] = replace(
            cal, homing_offset=offset, range_min=low + delta, range_max=high + delta
        )
    return centered
