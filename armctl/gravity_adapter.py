"""``armctl_openyam_damiao``: DimOS's OpenYAM Damiao adapter with a per-joint gravity-feedforward scale.

See ``armctl/gravity.py`` for why. This subclasses DimOS 0.0.14's ``OpenYamDamiaoAdapter`` and
overrides two hooks only:

* ``_gravity_torques()``: DimOS's vector (pinocchio ``computeGeneralizedGravity`` at the measured
  q, or zeros when ``gravity_comp`` is off) times the per-joint scale. DimOS calls it from
  ``write_motor_commands`` (``tau + gravity``) and ``_preflight_gravity`` (``activate``).
* ``kinematic_model``: the stock ``urdf/yam_gripper_gravity.urdf`` unless a validated
  replacement was given (same joints and limits, see ``gravity.check_gravity_urdf``).

Everything else (CAN, limits, fault latching, gripper) is DimOS's code. Constructed by
``ArmctlTwistCoordinator._create_whole_body_adapter`` in the DimOS worker.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

from armctl import gravity as _g
from dimos.hardware.whole_body.openyam_damiao.adapter import OpenYamDamiaoAdapter
from dimos.robot.assets.model import RobotModel


class ArmctlOpenYamDamiaoAdapter(OpenYamDamiaoAdapter):
    """OpenYamDamiaoAdapter + ``armctl_gravity={"scale": (6 floats), "urdf": str | None}``."""

    def __init__(self, *args, armctl_gravity: dict | None = None, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        req = armctl_gravity or {}
        self._armctl_scale = np.asarray(_g.check_vector(req.get("scale") or _g.UNITY), dtype=np.float64)
        urdf = req.get("urdf")
        if urdf:
            probs = _g.check_gravity_urdf(urdf, _g.stock_urdf())
            if probs:
                raise ValueError(f"--gravity-urdf {urdf} refused: " + "; ".join(probs))
        self._armctl_urdf = Path(urdf) if urdf else None

    @property
    def armctl_gravity_scale(self) -> tuple[float, ...]:
        return tuple(float(x) for x in self._armctl_scale)

    @property
    def kinematic_model(self) -> RobotModel:
        if self._armctl_urdf is not None:
            return RobotModel.from_file(self._armctl_urdf)
        return super().kinematic_model

    def _gravity_torques(self) -> np.ndarray:
        g = super()._gravity_torques()
        if g.shape != self._armctl_scale.shape:
            raise ValueError(f"gravity vector {g.shape} vs scale {self._armctl_scale.shape}")
        return g * self._armctl_scale


def offline_gravity(adapter: ArmctlOpenYamDamiaoAdapter, q) -> np.ndarray:
    """The feedforward this adapter would add at joint positions ``q``, with no CAN: loads the
    kinematic model and reads ``q`` instead of the motors. For previews and tests only."""
    adapter._load_kinematic_model()
    qq = np.asarray(q, dtype=np.float64)
    adapter._arm_positions = lambda: qq  # instance attribute shadows the motor read
    return adapter._gravity_torques()
