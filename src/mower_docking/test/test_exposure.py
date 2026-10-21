from mower_docking.exposure import AUTO, STOCK_SEQUENCE, ExposureStepper


def feed(s, t0, t1, det, dt=0.1, **kw):
    out = []
    t = t0
    while t < t1 - 1e-9:
        c = s.on_frame(t, det(t) if callable(det) else det, **kw)
        if c is not None:
            out.append((round(t, 2), c))
        t += dt
    return out


def test_first_frame_day_is_auto():
    s = ExposureStepper()
    assert s.on_frame(0.0, False, brightness=120.0) == AUTO


def test_first_frame_dark_starts_manual_preset():
    s = ExposureStepper(dark_brightness=50, dark_start=30)
    assert s.on_frame(0.0, False, brightness=20.0) == 30


def test_steps_through_stock_sequence_and_wraps():
    s = ExposureStepper(window_s=3.0, min_frames=10)
    s.on_frame(0.0, False, brightness=100.0)
    cmds = feed(s, 0.1, 3.5 * 11, False)
    vals = [c for _, c in cmds]
    assert vals[:len(STOCK_SEQUENCE)] == list(STOCK_SEQUENCE)
    assert vals[len(STOCK_SEQUENCE)] == AUTO          # wraps back to auto
    # each step needs a full window (+ settle)
    assert cmds[1][0] - cmds[0][0] >= 3.0


def test_good_ratio_keeps_setting():
    s = ExposureStepper()
    s.on_frame(0.0, False, brightness=100.0)
    assert feed(s, 0.1, 20.0, lambda t: int(t * 10) % 3 == 0) == []   # ratio 0.33 > 0.2


def test_ratio_at_threshold_steps():
    s = ExposureStepper()
    s.on_frame(0.0, False, brightness=100.0)
    cmds = feed(s, 0.1, 4.0, lambda t: int(round(t * 10)) % 5 == 0)   # 20 %
    assert cmds and cmds[0][1] == 100


def test_inactive_does_not_step():
    s = ExposureStepper()
    s.on_frame(0.0, False, brightness=100.0)
    assert feed(s, 0.1, 20.0, False, active=False) == []


def test_lost_steps_every_second():
    s = ExposureStepper(lost_step_s=1.0)
    s.on_frame(0.0, True, brightness=100.0)
    cmds = feed(s, 0.1, 3.15, False, lost=True)
    assert [c for _, c in cmds] == [100, 90, 80]


def test_stop_restores_auto_only_if_changed():
    s = ExposureStepper()
    s.on_frame(0.0, True, brightness=100.0)
    assert s.stop() is None
    s.on_frame(0.0, False, brightness=10.0)
    assert s.stop() == AUTO
    assert s.setting is None


def test_disabled_never_commands():
    s = ExposureStepper(enabled=False)
    assert s.on_frame(0.0, False, brightness=10.0) is None
    assert feed(s, 0.1, 10.0, False, lost=True) == []
    assert s.stop() is None
