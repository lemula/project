DURATION = 10.0

T0 = 0.0
T1 = 3.0
T2 = 8.0
T3 = 10.0

TARGETS_SEGMENTS = {
    "steer": (0.0, -30.0, 0.0),
    "boom": (0.0, 40.0, 0.0),
    "bucket": (0.0, 10.0, 0.0),
}


def get_targets(t):
    if t < T1:
        return {"steer": 0.0, "boom": 0.0, "bucket": 0.0}
    if t < T2:
        return {"steer": -30.0, "boom": 40.0, "bucket": 10.0}
    if t < T3:
        return {"steer": 0.0, "boom": 0.0, "bucket": 0.0}
    return {"steer": 0.0, "boom": 0.0, "bucket": 0.0}
