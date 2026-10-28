"""Holding-gain override tests (pure; no zenoh, no coordinator start).

Run: .venv/bin/python -m pytest armctl/tests/test_gains.py
"""
import dataclasses
import importlib
import sys

import pytest

from armctl import gains as g


def test_parse_basic_and_repeatable():
    assert g.parse_overrides(None, "kp") == {}
    assert g.parse_overrides(["j4=22"], "kp") == {"j4": 22.0}
    assert g.parse_overrides(["j4=22", "j5=15"], "kp") == {"j4": 22.0, "j5": 15.0}
    assert g.parse_overrides(["j4=22,j6=12"], "kp") == {"j4": 22.0, "j6": 12.0}
    assert g.parse_overrides([" J4 = 2 "], "kd") == {"j4": 2.0}
    assert g.parse_overrides(["yam_joint4=2.5"], "kd") == {"j4": 2.5}


@pytest.mark.parametrize("spec", ["j4", "j4:22", "j7=1", "gripper=1", "j0=1", "j4=abc", "j4=nan", "j4=inf", "=3"])
def test_parse_rejects_malformed(spec):
    with pytest.raises(ValueError):
        g.parse_overrides([spec], "kp")


def test_parse_rejects_duplicates_and_bad_kind():
    with pytest.raises(ValueError, match="twice"):
        g.parse_overrides(["j4=20", "j4=22"], "kp")
    with pytest.raises(ValueError):
        g.parse_overrides(["j4=20"], "ki")


def test_caps():
    # kp: wrist (0, 40], j1-j3 (0, 120]
    assert g.parse_overrides(["j4=40", "j1=120"], "kp") == {"j4": 40.0, "j1": 120.0}
    for bad in ("j4=40.1", "j5=41", "j6=100", "j1=120.5", "j3=200", "j4=0", "j2=-5"):
        with pytest.raises(ValueError, match="refused"):
            g.parse_overrides([bad], "kp")
    # kd: wrist [0, 5], j1-j3 [0, 8]
    assert g.parse_overrides(["j4=5", "j2=8", "j6=0"], "kd") == {"j4": 5.0, "j2": 8.0, "j6": 0.0}
    for bad in ("j4=5.5", "j6=10", "j1=9", "j4=-0.1"):
        with pytest.raises(ValueError, match="refused"):
            g.parse_overrides([bad], "kd")


def test_apply_overrides_keeps_defaults_and_gripper():
    kp = g.apply_overrides(g.DIMOS_DEFAULT_KP, {"j4": 22.0})
    assert kp == (80.0, 80.0, 80.0, 22.0, 10.0, 10.0, 0.0)
    assert g.apply_overrides(g.DIMOS_DEFAULT_KD, {}) == g.DIMOS_DEFAULT_KD


def _openyam_sim_hw():
    from dimos.core.global_config import global_config
    from dimos.robot.manipulators.openyam.config import openyam_hardware

    global_config.update(simulation="1", can_port=None)
    return openyam_hardware()


def test_dimos_defaults_match_what_we_document():
    hw = _openyam_sim_hw()
    assert hw.adapter_type == "mock_whole_body"
    assert tuple(hw.wb_config.kp) == g.DIMOS_DEFAULT_KP
    assert tuple(hw.wb_config.kd) == g.DIMOS_DEFAULT_KD


def test_override_wb_config_on_real_dimos_component():
    hw = _openyam_sim_hw()
    orig = hw.wb_config
    new = g.override_wb_config(orig, {"j4": 22.0}, {"j4": 2.0})
    assert type(new) is type(orig) and dataclasses.is_dataclass(new)
    assert new.kp == (80.0, 80.0, 80.0, 22.0, 10.0, 10.0, 0.0)
    assert new.kd == (5.0, 5.0, 5.0, 2.0, 1.5, 1.5, 0.0)
    assert len(new.kp) == len(hw.joints) and len(new.kd) == len(hw.joints)
    assert orig.kp == g.DIMOS_DEFAULT_KP  # original untouched (frozen)
    # no overrides -> the very same object (defaults left alone)
    assert g.override_wb_config(orig, {}, {}) is orig
    # kd-only override leaves kp alone
    only_kd = g.override_wb_config(orig, {}, {"j5": 1.0})
    assert only_kd.kp == g.DIMOS_DEFAULT_KP and only_kd.kd[4] == 1.0
    with pytest.raises(ValueError):
        g.override_wb_config(None, {"j4": 22.0}, {})


def test_gains_dict_and_describe():
    from dimos.hardware.whole_body.spec import WholeBodyConfig

    wb = WholeBodyConfig(kp=g.apply_overrides(g.DIMOS_DEFAULT_KP, {"j4": 22.0}),
                         kd=g.apply_overrides(g.DIMOS_DEFAULT_KD, {"j4": 2.0}))
    d = g.gains_dict(wb)
    assert d["kp"]["j4"] == 22.0 and d["kd"]["j4"] == 2.0 and d["kp"]["gripper"] == 0.0
    assert g.describe(wb) == "kp=[80,80,80,22,10,10] kd=[5,5,5,2,1.5,1.5]"
    assert g.gains_dict(None) == {"kp": None, "kd": None}


def test_coordinator_cli_refuses_out_of_cap_gains(monkeypatch):
    """main() must refuse bad gains during argument checks, before pgrep or any coordinator start.

    The import is done on the sim/test bus so it can never default to the real-arm bus.
    """
    monkeypatch.setenv("ZENOH_SCOUT_ADDR", "224.0.0.124:7450")
    monkeypatch.delenv("ARMCTL_SCOUT_DEFAULTED", raising=False)
    monkeypatch.delenv("CAN_PORT", raising=False)
    sys.modules.pop("armctl.dimos_coordinator", None)
    dc = importlib.import_module("armctl.dimos_coordinator")

    def boom(*a, **k):  # pragma: no cover - reaching this is the failure
        raise AssertionError("main() went past argument checks")

    monkeypatch.setattr(dc, "_other_dimos_running", boom)
    monkeypatch.setattr(dc, "build", boom)
    for argv in (["--simulation", "--kp", "j4=50"], ["--simulation", "--kd", "j4=6"],
                 ["--simulation", "--kp", "j1=121"], ["--simulation", "--kp", "gripper=5"],
                 ["--simulation", "--kp", "j4=20", "--kp", "j4=22"]):
        with pytest.raises(SystemExit) as e:
            dc.main(argv)
        assert e.value.code == 2
