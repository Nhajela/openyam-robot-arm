"""Board vision v1 for chess on a real board (see README.md and interfaces/grasp/CHESS.md).

- ``sensor``: the shared ``Snapshot`` / ``BoardSensor`` interface used by the game.
- ``board_geom``: board localisation from four ArUco corner markers, top-down warp.
- ``occupancy``: per-square occupancy and piece colour against an empty-board reference.
- ``camera``: ``CameraBoardSensor``, multi-camera fusion behind ``snapshot()``.
- ``calibrate`` / ``watch``: CLIs. ``synth``: the synthetic renderer the tests use.
"""
