"""CtrlRateKeeper: controller_frequency re-sent on every transition into active."""
from mower_sim.ctrl_rate import CtrlRateKeeper

UNCONFIGURED, INACTIVE, ACTIVE = 1, 2, 3
CONFIGURING, ACTIVATING, DEACTIVATING = 10, 13, 14


def keeper():
    sent = []
    return CtrlRateKeeper(sent.append, 60.0), sent


def test_set_again_after_pause_and_resume():
    # start-up: configure -> activate, the gate's PAUSE (deactivate), START resumes
    k, sent = keeper()
    for s in (UNCONFIGURED, CONFIGURING, INACTIVE, ACTIVATING, ACTIVE):
        k.observe(s)
    assert sent == [60.0]
    k.result(True)
    for s in (DEACTIVATING, INACTIVE, ACTIVATING, ACTIVE):
        k.observe(s)
    assert sent == [60.0, 60.0]
    assert k.activations == 2


def test_set_lost_in_the_pause_is_resent_on_resume():
    # the 2026-11-01 race: the set is sent while active, the pause lands first, the set
    # still reports success (the parameter changed, the loop rate did not)
    k, sent = keeper()
    k.observe(ACTIVE)
    k.observe(DEACTIVATING)
    k.result(True)
    k.observe(INACTIVE)
    k.observe(ACTIVE)
    assert sent == [60.0, 60.0]


def test_repeated_active_reports_send_once():
    k, sent = keeper()
    k.observe(ACTIVE)
    k.result(True)
    k.observe(ACTIVE)          # GetState poll after the transition event
    k.tick()
    assert sent == [60.0]


def test_failed_set_retried_while_active_only():
    k, sent = keeper()
    k.observe(ACTIVE)
    k.tick()                   # in flight: no second request
    assert sent == [60.0]
    k.result(False)            # e.g. refused while a goal runs
    k.tick()
    assert sent == [60.0, 60.0]
    k.result(False)
    k.observe(DEACTIVATING)
    k.tick()
    assert sent == [60.0, 60.0]


def test_nothing_sent_before_active():
    k, sent = keeper()
    assert not k.known
    for s in (UNCONFIGURED, INACTIVE):
        k.observe(s)
        k.tick()
    assert k.known and sent == []
