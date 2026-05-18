DURATION = 15.0

T0 = 0.0
T3 = 15.0

TARGETS_SEGMENTS = {
    "big_arm": (0.0, 15.0, 30.0, 45.0, 30.0, 15.0, 0.0),
    "small_arm": (150.0, 135.0, 120.0, 105.0, 120.0, 135.0, 150.0),
    "bucket": (65.0, 85.0, 110.0, 135.0, 110.0, 85.0, 65.0),
}

MULTI_AXIS = True
ACTIVE_AXIS = "big_arm"
START_VALUES = {}


def _blend(a, b, u):
    return a


def _interp_series(t, values):
    if len(values) == 1:
        return values[0]
    total = T3 - T0 if T3 > T0 else DURATION
    if t <= T0:
        return values[0]
    if t >= T0 + total:
        return values[-1]
    seg = total / (len(values) - 1)
    idx = int((t - T0) / seg)
    if idx >= len(values) - 1:
        return values[-1]
    t0 = T0 + idx * seg
    u = (t - t0) / max(seg, 1e-6)
    return _blend(values[idx], values[idx + 1], u)


def set_start_values(values):
    for key, val in values.items():
        if val is None:
            continue
        START_VALUES[key] = float(val)


def get_targets(t):
    if MULTI_AXIS:
        targets = {}
        for name, values in TARGETS_SEGMENTS.items():
            start_val = START_VALUES.get(name)
            if start_val is not None and len(values) > 0:
                vals = list(values)
                vals[0] = start_val
                values = vals
            targets[name] = _interp_series(t, values)
        return targets

    targets = {"big_arm": None, "small_arm": None, "bucket": None}
    values = TARGETS_SEGMENTS.get(ACTIVE_AXIS)
    if values is None:
        return targets
    start_val = START_VALUES.get(ACTIVE_AXIS)
    if start_val is not None and len(values) > 0:
        vals = list(values)
        vals[0] = start_val
        values = vals
    targets[ACTIVE_AXIS] = _interp_series(t, values)
    return targets
