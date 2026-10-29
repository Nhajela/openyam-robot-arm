# Reuse, don't reinvent: who has already solved detect / follow / pick (2026-10-29)

- **Experimenters:** Naman (question) + Claude (two research agents + checks)
- **Question (Naman, 29 Oct):** object detection, following and picking are solved problems. DimOS (Dimensional) and thousands of others must have done them. Find it and port it to our setup and constraints instead of inventing.
- **Method:** (1) a read of the installed DimOS 0.0.14 source (`~/src/openyam/dimos-venv/.../site-packages/dimos/`, plus the docs clone at `~/.local/share/dimos/repo`, commit c18e2fb); (2) web research (GitHub, docs, HF, papers, fetched 29 Oct). The main session re-checked the key claims marked ✅ in the source. Nothing was run, downloaded or moved.
- **Our constraints:**
  - one wrist RGB fisheye camera, no depth;
  - DimOS used as the motor coordinator only;
  - armctl must stay the only thing that sends motion;
  - the laptop GPU is unknown, and the DimOS venv has CPU-only torch (`torch-2.14.1+cpu`) ✅;
  - serenity has no GPU ✅.

## Bottom line
Nobody has published a YAM that picks or follows using only a wrist RGB camera. But DimOS, the stack we already run, contains almost all the pieces:

| Need | What already exists | Where | Inputs | Verdict for us |
|---|---|---|---|---|
| "Find the thing I name" | **YOLO-E** open-vocabulary detection + segmentation + track IDs (`Yoloe2DDetector`, text prompts or prompt-free) ✅ | `perception/detection/detectors/yoloe.py` | RGB; CUDA or CPU (auto) ✅ | **Adopt.** Weights (`models_yoloe`, ~185 MB) are not on disk yet: git-lfs pointers only ✅. Also OWLv2 and OmDet in the same folder (OWL-ViT ~11 s/prompt on CPU per DimOS docs). |
| "Follow it" | **Drone tracker:** Qwen VLM finds the box → OpenCV CSRT tracks → PID on the pixel error → velocity ✅ | `robot/drone/drone_tracking_module.py`, `drone_visual_servoing_controller.py`, `navigation/visual_servoing/visual_servoing_2d.py` | RGB, CPU | **The same pattern as our `interfaces/follow`.** Theirs is simpler: no latency compensation, no arm limits, publishes a drone Twist. Reuse the front end (named object → box → CSRT); keep our arm side (capture-time q, soft box, table guard, deadman). Nothing in DimOS follows with an *arm*. |
| "Track it every frame" | CSRT (`ObjectTracker2D`, CPU, 5 Hz loop) ✅; EdgeTAM (better, **CUDA/MPS only**) ✅ | `perception/experimental/object_tracker_2d.py`, `models/segmentation/edge_tam.py` | RGB | CSRT now; EdgeTAM if the laptop has a GPU. |
| "Pick it up" | **Agentic pick-and-place** (PR #1237, merged 18 Feb 2026): `scan_objects` → per-object point cloud → grasps → pregrasp → straight approach → close → verify by gripper readback → retract → `place_at` ✅ | `manipulation/pick_and_place_module.py` (~200 lines of logic), blueprints only for **xArm7 + RealSense** (`robot/manipulators/xarm/blueprints/grasp.py`) | **Depth mandatory** (point clouds from `depth_image`) | **Port with an adapter.** Feed it a synthetic object cloud: mask pixels → camera rays (our hand-eye + FK) → table plane + known object height. Its `HeuristicGrasp` (top-down at the median xy, yaw across the narrow axis) only needs that. Its own planner/executor must not drive the arm: wrap motion as armctl `move_joints`. |
| Grasp verify | `grasp_verification.py`: gripper readback vs command ✅ | | none | Same idea as our `grasp.holding`; usable as is. |
| Grasp poses (learned) | GraspGenX ✅ | `manipulation/grasping/grasp_gen_x.py` | point cloud + CUDA 12.8 GPU | Parked (depth + GPU). |
| "Pick up the X" by voice/agent | MCP: every `@skill` is a tool (port 9990); `xarm/blueprints/agentic.py` ✅ | | | Later: expose our wrapped skills the same way. |

OpenYAM in DimOS: only `coordinator-openyam` and `openyam_planner_coordinator` blueprints (no camera, no perception) ✅. No DimOS demo of a YAM picking or following was found (hackathons used the Go2).

## Outside DimOS (web, 29 Oct)

| Candidate | What | Fit | Effort |
|---|---|---|---|
| XLeRobot `3_so100_yolo_ee_control.py` | Training-free "follow the bottle": YOLO → end-effector control on an SO-100 | Same pattern, a reference (⚠️ docs/snippet) | S |
| LeRobot ACT / SmolVLA | Imitation learning; typical 20–50 episodes per task, usually wrist + a fixed camera; one wrist-only ACT card ~80% at a fixed position, out of distribution 10–30% | Second route to "pick the cup", needs teleop demos + a GPU to train; record through armctl | M |
| anvil-embodied-ai (Apache-2.0) | Recording → LeRobot v3 → ACT/Diffusion/SmolVLA/π0 → ROS2 inference | Borrow converter/configs for the learned route | M |
| MolmoAct2-BimanualYAM (Apache-2.0) | Open VLA trained on bimanual YAM; 3 cameras, 14-D, <16 GB VRAM | Not a 1–3 day option | L |
| YAM datasets (⚠️ snippets): Neotix `ball_to_cup` (456 ep), `insert-battery` (1,119 ep), `yam pick cube single` (19 ep) | Co-training data | Later | — |
| Gemini Robotics-ER 1.5/1.6 API | Points at a named object, predicts a 2D grasp point/angle | Alternative to Qwen/Claude for "find it" | S |
| ViSP | The visual-servoing reference (C++; Python via conda-forge) | Not needed: centring + known size is ~50 lines | — |
| Depth Anything V2 (metric) | Monocular depth; ~0.3–0.66 m mean error indoors in one study | Too coarse for closing a gripper; the table plane + known size is better | — |
| RGBManip (arXiv 2310.03478) | Wrist-only multi-view 6-D pose | Research code | L |

No off-the-shelf RGB-only grasp planner for a parallel gripper was found. The practical recipe is: mask → footprint on the calibrated table plane → top-down grasp from the known size. That is what a synthetic depth cloud gives DimOS's heuristic grasp.

## What this means for our code
- **Replace (invented, solved elsewhere):** the hand-built cup finder (rim circles) and tap segmentation, with YOLO-E (named object, mask); the patch tracker's role, with CSRT or YOLO-E track IDs (benchmark both against ours on CPU); the grasp state machine's pick logic, with DimOS `PickAndPlaceModule`'s sequence + `HeuristicGrasp`.
- **Keep (specific to our setup, not in DimOS for arms):** armctl (lease, deadman, gate); hand-eye/table calibration; capture-time q + latency handling; the arm-side follow controller (soft box, table guard, reach limits).

## People to ask
1. Dimensional Discord (discord.gg/dimos), **mustafab0** (author of PR #1237): RGB-only depth source or a YAM perception blueprint planned?
2. Anvil Robotics (Mike Xia): OpenYAM pick datasets/policies?
3. i2rt GitHub issues; Neotix Robotics (YAM Pro datasets).
4. LeRobot Discord / XLeRobot: the YOLO follower; small-data ACT/SmolVLA.

## Not verified
- YOLO-E speed and accuracy on CPU on our oblique fisheye view.
- Moondream on CPU.
- ~~Which OpenCV wins in the DimOS venv~~ checked: 4.14.0 (contrib), `cv2.legacy.TrackerCSRT_create` present; the repo `.venv` (5.0.0) has `cv2.TrackerCSRT_create` too.
- Whether `PickAndPlaceModule` can be built outside a blueprint with injected specs (else copy its logic).
- The laptop's GPU.
- The HF dataset details (snippets only).
