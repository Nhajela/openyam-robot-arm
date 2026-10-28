"""Moves authored for dance caps are slowed to the daemon's caps instead of refused (28 Oct, real arm)."""
from armctl import envelope as env
from armctl import playable as P
from openplay import moves as M
from openplay.server import fit_to_caps, playable_info


def _env(vel, acc):
    return env.Envelope(vel_caps=vel, acc_caps=acc).describe()


def test_dance_move_is_slowed_to_default_caps():
    for name in ("robot-dance", "peekaboo"):
        p = M.load(name)
        assert p.get("caps") == "dance"
        fitted, slowed = fit_to_caps(p, _env(env.VEL_CAP, env.ACC_CAP))
        assert slowed and fitted["caps"] == "default"
        assert P.validate(fitted)["ok"], P.validate(fitted)
        assert playable_info(fitted)[0] > playable_info(p)[0]


def test_untouched_when_daemon_runs_dance_caps():
    p = M.load("robot-dance")
    fitted, slowed = fit_to_caps(p, _env(0.6, 2.0))
    assert not slowed and fitted is p
