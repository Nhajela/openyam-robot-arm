"""Gravity-feedforward scale tests (no zenoh, no coordinator start, no CAN: adapters are only
constructed, motor IO is faked).

Run: .venv/bin/python -m pytest armctl/tests/test_gravity.py
"""
import importlib
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from armctl import gravity as g

# ---- parsing --------------------------------------------------------------------------------


def test_parse_scales():
    assert g.parse_scales(None) == {}
    assert g.parse_scales(["j4=0.83"]) == {"j4": 0.83}
    assert g.parse_scales(["j4=0.83", "j2=1.1"]) == {"j4": 0.83, "j2": 1.1}
    assert g.parse_scales(["j2=1.1,j3=1.1"]) == {"j2": 1.1, "j3": 1.1}
    assert g.parse_scales(["yam_joint4=0.5", "J6 = 1.5"]) == {"j4": 0.5, "j6": 1.5}  # bounds inclusive


@pytest.mark.parametrize("spec", ["j4=0.49", "j4=1.51", "j4=0", "j4=-0.83", "j4=nan", "j4=inf", "j2=2"])
def test_parse_refuses_out_of_range(spec):
    with pytest.raises(ValueError, match="refused"):
        g.parse_scales([spec])


@pytest.mark.parametrize("spec", ["j4", "j4:0.8", "j7=1", "gripper=1", "j4=abc", "=1"])
def test_parse_refuses_malformed(spec):
    with pytest.raises(ValueError):
        g.parse_scales([spec])


def test_parse_refuses_duplicates():
    with pytest.raises(ValueError, match="twice"):
        g.parse_scales(["j4=0.8", "j4=0.9"])


def test_vector_and_check():
    assert g.scale_vector({}) == g.UNITY
    assert g.scale_vector({"j4": 0.83}) == (1.0, 1.0, 1.0, 0.83, 1.0, 1.0)
    for bad in ((1.0,) * 5, (1.0, 1.0, 1.0, 1.6, 1.0, 1.0), (1.0, 1.0, 1.0, 0.3, 1.0, 1.0)):
        with pytest.raises(ValueError):
            g.check_vector(bad)
    assert g.request({}, None) is None


# ---- hardware component ------------------------------------------------------------------------


@pytest.fixture
def dimos_hw():
    """openyam_hardware() as DimOS builds it for sim or real (can_port None: never a CAN name)."""
    from dimos.core.global_config import global_config
    from dimos.robot.manipulators.openyam.config import openyam_hardware

    old = (global_config.simulation, global_config.can_port)

    def make(sim: bool):
        global_config.update(simulation="1" if sim else "", can_port=None)
        return openyam_hardware()

    yield make
    global_config.update(simulation=old[0], can_port=old[1])


def test_defaults_unchanged(dimos_hw):
    for sim in (True, False):
        hw = dimos_hw(sim)
        assert g.apply_to_hardware(hw, {}, None) is hw
        assert g.apply_to_hardware(hw, None, None) is hw
        assert g.KWARG not in hw.adapter_kwargs
        assert g.info(hw)["scale"] is None and not g.info(hw)["applied"]


def test_real_component_gets_armctl_adapter(dimos_hw):
    hw = dimos_hw(False)
    assert hw.adapter_type == g.STOCK_ADAPTER
    new = g.apply_to_hardware(hw, {"j4": 0.83}, None)
    assert new.adapter_type == g.ARMCTL_ADAPTER
    assert new.adapter_kwargs[g.KWARG] == {"scale": (1.0, 1.0, 1.0, 0.83, 1.0, 1.0), "urdf": None}
    assert new.adapter_kwargs["runtime_config"] is hw.adapter_kwargs["runtime_config"]  # DimOS's kwargs kept
    assert new.wb_config == hw.wb_config and new.joints == hw.joints
    assert hw.adapter_type == g.STOCK_ADAPTER and g.KWARG not in hw.adapter_kwargs  # original untouched
    info = g.info(new)
    assert info["applied"] and info["scale"]["j4"] == 0.83 and info["note"] is None


def test_sim_component_keeps_mock_and_records(dimos_hw):
    hw = dimos_hw(True)
    new = g.apply_to_hardware(hw, {"j4": 0.83}, None)
    assert new.adapter_type == g.SIM_ADAPTER
    info = g.info(new)
    assert not info["applied"] and info["scale"]["j4"] == 0.83 and "mock" in info["note"]
    req, rest = g.split_kwargs(new)
    assert req["scale"][3] == 0.83 and g.KWARG not in rest


def test_unknown_adapter_refused(dimos_hw):
    import dataclasses

    hw = dataclasses.replace(dimos_hw(True), adapter_type="something_else")
    with pytest.raises(ValueError):
        g.apply_to_hardware(hw, {"j4": 0.9}, None)


# ---- the adapter subclass (constructed only, no connect) ----------------------------------------

POSES = [(0.0, 0.5, 0.8, j4, 0.0, 0.0) for j4 in (0.0, -0.3, 0.3, 0.7)]


def _adapters(req):
    from armctl.gravity_adapter import ArmctlOpenYamDamiaoAdapter
    from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
    from dimos.hardware.whole_body.openyam_damiao.adapter import OpenYamDamiaoAdapter

    cfg = DamiaoRuntimeConfig(gravity_comp=True)
    return OpenYamDamiaoAdapter(runtime_config=cfg), ArmctlOpenYamDamiaoAdapter(runtime_config=cfg, armctl_gravity=req)


def _dimos_gravity(adapter, q):
    adapter._load_kinematic_model()
    qq = np.asarray(q, float)
    adapter._arm_positions = lambda: qq
    return adapter._gravity_torques()


@pytest.mark.parametrize("q", POSES)
def test_unity_matches_dimos_and_scale_applies_per_joint(q):
    from armctl.gravity_adapter import offline_gravity

    stock, ours = _adapters(None)
    ref = _dimos_gravity(stock, q)
    np.testing.assert_allclose(offline_gravity(ours, q), ref, atol=1e-12)  # default = DimOS exactly
    _, scaled = _adapters(g.request({"j4": 0.83, "j2": 1.1}, None))
    out = offline_gravity(scaled, q)
    np.testing.assert_allclose(out, ref * np.array([1, 1.1, 1, 0.83, 1, 1]), atol=1e-12)


def test_ready_values_match_the_log():
    stock, new = g.preview({"j4": 0.83}, None)
    assert stock[3] == pytest.approx(2.378, abs=0.002)  # LOG.md 13:07 table
    assert new[3] == pytest.approx(0.83 * stock[3])
    assert new[2] == pytest.approx(stock[2])


def test_adapter_refuses_bad_scale_and_gravity_off_stays_zero():
    from armctl.gravity_adapter import ArmctlOpenYamDamiaoAdapter, offline_gravity
    from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig

    with pytest.raises(ValueError, match="refused"):
        ArmctlOpenYamDamiaoAdapter(armctl_gravity={"scale": (1, 1, 1, 1.6, 1, 1), "urdf": None})
    off = ArmctlOpenYamDamiaoAdapter(runtime_config=DamiaoRuntimeConfig(gravity_comp=False),
                                     armctl_gravity=g.request({"j4": 0.83}, None))
    assert not offline_gravity(off, POSES[0]).any()


class _FakeArm:
    def __init__(self):
        self.rows = None

    def mit_control(self, rows):
        self.rows = np.array(rows)


class _FakeGripper:
    def set_opening(self, x):
        self.opening = x


class _FakeRobot:
    def tick(self, deadline):
        self.ticked = deadline


def test_write_motor_commands_sends_scaled_feedforward():
    """DimOS's own write_motor_commands (MIT rows kp, kd, q, dq, tau + gravity) with fake motors."""
    from armctl.gravity_adapter import offline_gravity
    from dimos.hardware.whole_body.spec import MotorCommand

    q = POSES[1]
    stock, ours = _adapters(g.request({"j4": 0.83}, None))
    ref = _dimos_gravity(stock, q)
    offline_gravity(ours, q)
    ours._arms = {"arm": _FakeArm()}
    ours._grippers = {"gripper": _FakeGripper()}
    ours._robot = _FakeRobot()
    ours._connected = ours._active = True
    cmds = [MotorCommand(q=x, dq=0.0, kp=10.0, kd=1.5, tau=0.1) for x in q] + [MotorCommand(q=0.5)]
    assert ours.write_motor_commands(cmds) is True
    rows = ours._arms["arm"].rows
    np.testing.assert_allclose(rows[:, 4], 0.1 + ref * np.array([1, 1, 1, 0.83, 1, 1]), atol=1e-12)
    np.testing.assert_allclose(rows[:, 0], 10.0)  # kp untouched
    np.testing.assert_allclose(rows[:, 2], q)  # targets untouched


# ---- --gravity-urdf validation -----------------------------------------------------------------


def _edit(src, dst, fn):
    tree = ET.parse(str(src))
    fn(tree.getroot())
    tree.write(str(dst))
    return dst


def _link6_mass(root, value):
    for link in root.iter("link"):
        if link.get("name") == "yam_link_6":
            link.find("inertial/mass").set("value", str(value))


def test_gravity_urdf_check(tmp_path):
    stock = g.stock_urdf()
    assert g.check_gravity_urdf(stock, stock) == []
    lighter = _edit(stock, tmp_path / "light.urdf", lambda r: _link6_mass(r, 0.45))
    assert g.check_gravity_urdf(lighter, stock) == []

    def loosen_j1(r):
        for j in r.iter("joint"):
            if j.get("name") == "yam_joint1":
                j.find("limit").set("upper", "3.1416")

    def move_j5(r):
        for j in r.iter("joint"):
            if j.get("name") == "yam_joint5":
                j.find("origin").set("xyz", "-0.08 -0.0395003 0")

    def drop_j6(r):
        for j in list(r.iter("joint")):
            if j.get("name") == "yam_hand_tcp_joint":
                r.remove(j)

    for name, fn, pat in (("lim", loosen_j1, "limits"), ("org", move_j5, "origin"),
                          ("heavy", lambda r: _link6_mass(r, 5.0), "total mass"), ("drop", drop_j6, "joint set")):
        bad = _edit(stock, tmp_path / f"{name}.urdf", fn)
        probs = g.check_gravity_urdf(bad, stock)
        assert probs and any(pat in p for p in probs), (name, probs)
    (tmp_path / "junk.urdf").write_text("not xml")
    assert g.check_gravity_urdf(tmp_path / "junk.urdf", stock)


def test_gravity_urdf_changes_feedforward(tmp_path):
    from armctl.gravity_adapter import ArmctlOpenYamDamiaoAdapter

    lighter = _edit(g.stock_urdf(), tmp_path / "light.urdf", lambda r: _link6_mass(r, 0.45))
    stock, new = g.preview({}, lighter, POSES[1])
    assert new[3] < stock[3] - 0.5 and new[0] == pytest.approx(stock[0], abs=1e-9)
    bad = _edit(g.stock_urdf(), tmp_path / "bad.urdf", lambda r: _link6_mass(r, 50.0))
    with pytest.raises(ValueError, match="gravity-urdf"):
        ArmctlOpenYamDamiaoAdapter(armctl_gravity={"scale": g.UNITY, "urdf": str(bad)})


# ---- coordinator wiring (import on the sim bus; nothing is started) ----------------------------


@pytest.fixture
def dc(monkeypatch):
    monkeypatch.setenv("ZENOH_SCOUT_ADDR", "224.0.0.124:7450")
    monkeypatch.delenv("ARMCTL_SCOUT_DEFAULTED", raising=False)
    monkeypatch.delenv("CAN_PORT", raising=False)
    sys.modules.pop("armctl.dimos_coordinator", None)
    return importlib.import_module("armctl.dimos_coordinator")


def test_cli_refuses_bad_gravity_before_anything_runs(dc, monkeypatch, tmp_path):
    def boom(*a, **k):  # pragma: no cover - reaching this is the failure
        raise AssertionError("main() went past argument checks")

    monkeypatch.setattr(dc, "_other_dimos_running", boom)
    monkeypatch.setattr(dc, "build", boom)
    bad = _edit(g.stock_urdf(), tmp_path / "bad.urdf", lambda r: _link6_mass(r, 50.0))
    for argv in (["--gravity-scale", "j4=1.6"], ["--gravity-scale", "j4=0.4"], ["--gravity-scale", "gripper=1"],
                 ["--gravity-scale", "j4=0.8", "--gravity-scale", "j4=0.9"], ["--gravity-scale", "j4"],
                 ["--gravity-urdf", str(bad)], ["--gravity-urdf", str(tmp_path / "missing.urdf")]):
        with pytest.raises(SystemExit) as e:
            dc.main(["--simulation", *argv])
        assert e.value.code == 2, argv


def test_cli_passes_gravity_to_build(dc, monkeypatch):
    seen = {}

    class Stop(Exception):
        pass

    def fake_build(**kw):
        seen.update(kw)
        raise Stop

    monkeypatch.setattr(dc, "_other_dimos_running", lambda: [])
    monkeypatch.setattr(dc, "build", fake_build)
    with pytest.raises(Stop):
        dc.main(["--simulation", "--gravity-scale", "j4=0.83"])
    assert seen["gravity_scale"] == {"j4": 0.83} and seen["gravity_urdf"] is None
    seen.clear()
    with pytest.raises(Stop):
        dc.main(["--simulation"])
    assert seen["gravity_scale"] == {} and seen["gravity_urdf"] is None


def test_build_defaults_and_override(dc, dimos_hw):
    kw = dict(ik_margin=0.1, joint_vel=0.3, traj_vel=0.5, twist_timeout=0.3, with_planner=False)
    dimos_hw(True)
    hw, _ = dc.build(**kw)
    assert hw.adapter_type == g.SIM_ADAPTER and g.KWARG not in hw.adapter_kwargs
    hw, _ = dc.build(**kw, gravity_scale={"j4": 0.83})
    assert hw.adapter_type == g.SIM_ADAPTER and hw.adapter_kwargs[g.KWARG]["scale"][3] == 0.83
    dimos_hw(False)  # real-mode component, can_port None (no CAN name anywhere)
    hw, _ = dc.build(**kw)
    assert hw.adapter_type == g.STOCK_ADAPTER and g.KWARG not in hw.adapter_kwargs
    hw, _ = dc.build(**kw, gravity_scale={"j4": 0.83})
    assert hw.adapter_type == g.ARMCTL_ADAPTER


def test_create_whole_body_adapter_routing(dc, dimos_hw, monkeypatch):
    from armctl.gravity_adapter import ArmctlOpenYamDamiaoAdapter
    from dimos.control.coordinator import ControlCoordinator

    coord = dc.ArmctlTwistCoordinator.__new__(dc.ArmctlTwistCoordinator)  # no __init__: nothing starts
    passed = []
    monkeypatch.setattr(ControlCoordinator, "_create_whole_body_adapter",
                        lambda self, comp: passed.append(comp) or "dimos-factory")
    real = g.apply_to_hardware(dimos_hw(False), {"j4": 0.83}, None)
    ad = coord._create_whole_body_adapter(real)
    assert isinstance(ad, ArmctlOpenYamDamiaoAdapter) and ad.armctl_gravity_scale[3] == 0.83
    assert not ad.is_connected() and passed == []
    sim = g.apply_to_hardware(dimos_hw(True), {"j4": 0.83}, None)
    assert coord._create_whole_body_adapter(sim) == "dimos-factory"
    assert g.KWARG not in passed[-1].adapter_kwargs  # the mock never sees our kwarg
    stock = dimos_hw(False)
    assert coord._create_whole_body_adapter(stock) == "dimos-factory" and passed[-1] is stock
