"""armctl: one instruction layer and safety envelope for the OpenYAM arm.

Modules:
  envelope   joint limits, soft box, speed caps, named poses, check/clamp functions (stable API)
  backend    Backend interface, MockBackend (kinematic sim), DimosBackend (DimOS coordinator; sim-tested)
  kin        FK / IK / Jacobians in plain numpy (no pinocchio)
  daemon     WebSocket server (ws://127.0.0.1:8091/armctl): lease, deadman, gate, logging

See armctl/README.md and experiments/2026-10-16-linux-real-arm/ARMCTL-SPEC.md.
"""
