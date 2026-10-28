# Prior art: what we can reuse instead of building (OpenYAM / DimOS / armctl)

- **Experimenters:** Naman (+ Claude research agent)
- **Date:** 2026-10-28, 16:26 IST
- **Goal:** find existing repos, models, datasets, papers and people on the same or nearby infrastructure (i2rt YAM, Damiao motors, DimOS) that solve our current problems: j4 wrist holding error, wrist-camera calibration, cup grasping, chess, phone teleop, LLM control.
- **Status:** done. Desk research only: web search and fetch. Nothing touched the arm, CAN, cameras, zenoh or local ports.
- **Method / confidence:** each item was fetched or searched on 2026-10-28. ✅ means checked on the primary page (repo or official doc). ⚠️ means seen only in search snippets, mirrors or secondary coverage. Star counts were read from the pages on that day. Many GitHub pages did not show a last-commit date. The research agent wrote this text; the main session saved it here unchanged except for this note.

Effort: **S** = under a day, **M** = 2–5 days, **L** = a week or more, or needs a GPU or new hardware.

## 1. YAM / i2rt / Anvil / DimOS ecosystem

| Name | URL | What it is | Licence / activity | How we'd use it (effort) |
|---|---|---|---|---|
| **i2rt SDK** ✅ | github.com/i2rt-robotics/i2rt | Official YAM SDK: CAN control, gravity-comp floating mode, leader–follower with a teaching handle, MuJoCo/Viser sim, URDF/MJCF per variant, gripper auto-calibration, record/replay | MIT, 178★, 107 forks, 10 open issues; PR #40 (gravity + Coulomb compensation) merged 6 May 2026 | DimOS's openyam gains are i2rt's: `yam_v1.yml` has kp [80,80,80,10,10,10], kd [5,5,5,1.5,1.5,1.5]. The same file sets `gravity_comp_factor [1.0,1.1,1.1,1.2,1.0,1.0]`, `coulomb_friction [0.3,0.3,0.3,0.06,0.06,0.06]` and `grav_comp_kd [.1,.1,.1,.3,.05,.05]`. i2rt *raises* J4 gravity by 1.2×, but our data say DimOS's J4 feedforward is about 40% too strong. Diff i2rt's MJCF and crank_4310 gripper mass (0.604 kg at the flange) against DimOS's `yam_gripper_gravity.urdf`: the mismatch is probably mass or centre of mass, not the motors. Use their per-joint factor list as the shape of an armctl-side gravity scale. (S) |
| **i2rt issue #18** ✅ | github.com/i2rt-robotics/i2rt/issues/18 | "Gravity Compensation Drift & Joint Stiffness." Maintainer (22 Dec 2025): "occasionally seen J4 drift", caused by the imprecise Damiao current loop, "hence ×1.3 gravity factor". Tune dynamics per unit. No calibration script exists. | Reopened 15 Mar 2026 | Confirms our j4 symptom is known upstream. Post our j4 hold-test table there; it is better data than anything in the thread. (S) |
| **i2rt crank_4310.yml** ✅ | …/i2rt/robots/config/crank_4310.yml | Gripper config: DM4310 kp 20 / kd 0.5; `gripper_limits: null`, so it auto-calibrates at runtime; close 8° / open 170°; stroke 71 mm; force-limiter "clog" thresholds | MIT | Explains the start-up close/open sweep. Copy stroke and angles into `workcell.json`, and read the force limiter before we grip a cup. (S) |
| **Anvil OpenYAM_description** ✅ (repo list only) | github.com/anvil-robotics/OpenYAM_description | URDF/xacro for OpenYAM | 1★, updated Feb 2026 | Anvil's own masses for our build; compare with the gravity URDF for j4. (S) |
| **Anvil anvil-gripper-hw** ✅ (repo list only) | github.com/anvil-robotics/anvil-gripper-hw | Gripper hardware | CERN-OHL-S-2.0, updated 27 Oct 2026 | Check for the gripper mass and a camera-mount drawing (a prior for hand-eye). (S) |
| **Anvil anvil-embodied-ai** ✅ | github.com/anvil-robotics/anvil-embodied-ai | ROS2 MCAP recordings → LeRobot v3 dataset → ACT / Diffusion / SmolVLA / π0 / π0.5 → ROS2 inference node | Apache-2.0, 14★, Aug 2026 | Built for Devbox/OpenARM over ROS2; does not name OpenYAM or i2rt. Borrow the converter and training configs; ask Anvil about OpenYAM. (M) |
| **DimOS** ✅ | github.com/dimensionalOS/dimos | Manipulators in the tree: openyam, dual_openyam, xarm, piper, openarm, a1z, a750. McpServer/McpClient, `dimos mcp list-tools`, `dimos agent-send` | ~4.7k★, 835 forks, 232 open issues; pre-release beta; licence Apache-2.0 per third-party sources ⚠️ | PR #1225 "agentic pick-and-place" plus Pinocchio-IK teleop for xArm/Piper ⚠️ (seen via a mirror). Check whether it carries over to openyam. (S read, M adopt) |
| **robots_realtime** ✅ | github.com/uynitsuj/robots_realtime | Real-time teleop, data collection and policy deployment for bimanual i2rt YAM and Franka; ZeroMQ, Viser replay, Quest | 78★, 218 commits; licence not shown | Read how they run policy inference against the control loop and sync cameras. (S) |
| **GELLO** ✅ | github.com/wuphilipp/gello_software | Low-cost leader arm. YAM is listed first; `generate_yam_config.py` and `--use-save-interface` for recording | Berkeley (Philipp Wu) | The best input for learned-policy demos. Anvil lists a GELLO kit as "coming soon". (L) |
| **lerobot-yam** ⚠️ | pypi.org/project/lerobot-yam | LeRobot plugins: yam_follower, yam_leader (GELLO), crank_4310 | v0.1.0; page did not load | Goes through i2rt directly, so it **bypasses armctl**. Better to write an armctl-backed LeRobot Robot class. (M) |

## 2. Gravity compensation & gains (j4)

- **Upstream practice:** i2rt uses per-joint gravity factors (J4 = 1.2), a Coulomb feedforward and a separate damping term for gravity mode. Maintainers blame the Damiao current loop and advise per-unit tuning. Nobody has published an identification procedure for YAM.
- **Our data disagree in sign with i2rt:** DimOS gives J4 too much torque. The likely cause is that `yam_gripper_gravity.urdf` models a heavier gripper and camera, or a centre of mass further out, than we have.
- **Proposed next steps:**
  - (a) Fit a single scale `s` with τ_needed = s·g_model(q) from j4 runs 1 and 2, which we already have. If we want more data, hold with feedforward only (sim first).
  - (b) Add a per-joint `gravity_comp_factor` the way i2rt does. This is S if DimOS exposes a hook, otherwise M (a patch at `damiao/adapter.py:346-364`).
- **Generic method:** a least-squares fit of payload mass and centre of mass from static holds, τ = Y(q)·[m, m·c]. This is textbook; there is no YAM-specific tool. Trossen's Interbotix gravity-comp docs ⚠️ show a similar per-joint workflow.

## 3. Wrist / eye-in-hand calibration

| Name | URL | Why | Use (effort) |
|---|---|---|---|
| **mrcal** ✅ (docs) | mrcal.secretsauce.net; Debian package `mrcal` | Wide/fisheye splined model, projection-uncertainty maps and a valid-intrinsics region that shows the coverage gaps | Feed it our ChArUco corners (it takes external corner lists). Use the uncertainty map to direct the capture. Keep OpenCV at runtime. (S–M) |
| **OpenCV calibrateRobotWorldHandEye** ✅ | docs.opencv.org/4.x calib3d | Solves camera→gripper and board→base together; `handeye.py` does these as two steps | Add it as a sixth solver and compare consistency. (S) |
| **easy_handeye / easy_handeye2** ✅ | github.com/IFL-CAMP/easy_handeye, marcoesposito1988/easy_handeye2 | ROS. Its useful part for us is the sampling practice: wide rotations about all axes, 15+ samples | Copy its view-diversity rules into `plan_views`. (S) |
| **Online ChArUco hand-eye (Sensors 2022)** ✅ | ncbi.nlm.nih.gov/pmc/articles/PMC9147787 | ChArUco eye-in-hand at 0.4–0.6 mm | Background |
| **Piper hand-eye thread** ⚠️ | discourse.openrobotics.org/t/piper-hand-eye-calibration/51407 | A peer low-cost arm community solving the same problem | Read for pitfalls |

**Tips:**
- Keep the board rigid and flat. A lifting sheet ruins hand-eye (OpenCV forum).
- Hand-eye needs large rotations between stations, about different axes. With translation-only variety TSAI degenerates, which may be why our first hand-eye was rejected.
- With a small board and a wide lens, edge coverage only comes from close, tilted views. Do intrinsics as a separate hand-held pass (`hand_capture.py` already does this).
- OpenCV 4.6 changed how ChArUco boards with an even row count are generated (`setLegacyPattern`). Our 6-row board is generated and detected by the same code, so it should be consistent; check this if anyone prints it with another tool. ⚠️ from memory, not re-verified.
- Use the ~60° bracket angle as a prior and reject solutions far from it.

## 4. Grasping a known cup + learned policies

| Name | URL | Fit | Licence / maturity | Use (effort) |
|---|---|---|---|---|
| **Our CupFinder** | interfaces/wristcam/locate.py | For one known cup and a monocular camera, geometry plus servoing is the right tool. None of the grasp networks below take RGB only. | n/a | Keep it as primary |
| **Grounded-SAM-2** ⚠️ | github.com/IDEA-Research/Grounded-SAM-2 | Open-vocabulary cup box, mask and tracking | Licence unverified (a mirror says Apache-2.0); needs a GPU | A fallback `find()` behind the same Detection API. (M) |
| **NVIDIA GraspGenX** ✅ | github.com/NVlabs/GraspGenX | Cross-gripper 6-DoF grasps; custom gripper via URDF plus a wizard; MCP interface | Code Apache-2.0, weights NVIDIA Open Model License; 276★; released Jun 2026 | Needs depth or point clouds, so park it until we have a depth camera. (L) |
| **GraspGen** ✅ | github.com/NVlabs/GraspGen | Franka, Robotiq and suction only | Licence unclear, possibly non-commercial ⚠️ | Skip |
| **MolmoAct2 (AI2)** ✅ | github.com/allenai/molmoact2; hf.co/allenai/MolmoAct2 | Open VLA trained on 720 h of bimanual YAM. Checkpoints for BimanualYAM, SO100_101 and DROID; LeRobot has a MolmoAct2 policy | Open weights; arXiv 2605.02881 (May 2026) | The only foundation model pre-trained on our arm. Fine-tune 14-D bimanual down to 7-D single arm with our wrist camera; run on a GPU and stream through armctl's gate. (L) |
| **π0 / π0.5 / π0-FAST (openpi)** ⚠️ | github.com/Physical-Intelligence/openpi; YAM fine-tunes: hf.co/robocurve/pi05-yam-molmoact2, hf.co/angkul07/pi0-fast-yam, hf.co/YzyLmc/pi05_pick_block_red_yam | Mostly bimanual YAM. Trossen's openpi tutorial is a template for a single-arm config | Apache-2.0; A100-class GPU to fine-tune | An alternative to MolmoAct2; pick one. (L) |
| **LeRobot ACT / SmolVLA** ✅ | huggingface.co/docs/lerobot | Small, trains on a consumer GPU | Apache-2.0 | Anecdotes: an SO-100 tic-tac-toe pick reports 0.93–0.98 success (180 episodes); a 20-demo apple pick failed. Budget 50–200 demos. (M–L) |
| **YAM datasets** ⚠️ | hf.co/datasets/Dimios45/yam-pick-duster-200 (200 single-arm YAM pick-and-place demos, LeRobot v3, 2 cameras, 25 Hz); MolmoAct2-BimanualYAM (~5k episodes); YzyLmc/pick_block_red (30 episodes) | Format template and co-training data | Licences not checked | Copy the format for our recorder. (S) |

## 5. Chess robots and phone teleop

| Name | URL | Use (effort) |
|---|---|---|
| **chesscog** ✅ | github.com/georg-wolflein/chesscog (MIT, 148★) | Board corners → grid → occupancy CNN → 12-class piece CNN. Adapts to a new set from 2 start-position photos (paper: 99.8% per square; an independent test: 88.9% on unseen sets). Run it before and after "I've moved", diff, then match a legal move with python-chess. (M) |
| **ChessReD / end-to-end chess recognition** ✅ | arXiv 2310.04086 | A real-photo evaluation set. (S) |
| **Robby Fischer** ⚠️ | rerun.io/examples/robotics/chess_robby_fischer | Corner fiducials plus per-square occupancy and colour. Read for piece-grasp heights. (S) |
| **Hack Club SO-101 chess** ⚠️ | archive.hackclub.com | 3 cameras, homography, Stockfish; occupancy only. The minimal version. (S) |
| **LeRobot phone teleop** ✅ | huggingface.co/docs/lerobot/phone_teleop | iOS (ARKit via HEBI Mobile I/O) and Android (WebXR via `teleop`): 6-DoF phone pose drives the end-effector pose. Reuse their WebXR page and camera_offset handling as an openplay mode. (M) |
| **teleop-android** ⚠️ | github.com/giacomoran/teleop-android | ARCore app, better tracking than WebXR; WebSocket. (S) |

## 6. LLM / agent control

| Name | URL | Notes / use |
|---|---|---|
| **DimOS MCP** ✅/⚠️ | dimos repo | Exposes skills as MCP tools. Better: wrap **armctl's daemon** as an MCP server so Claude only gets gated, leased, E-STOP-able moves. (S–M) |
| **IliaLarchenko/robot_MCP** ✅ | github.com/IliaLarchenko/robot_MCP | Apache-2.0, 86★. MCP server plus agent for SO-100/101 that sends camera images to Claude, Gemini or GPT; the author rates Claude best. No safety layer. Copy its tool shapes and image budgeting. (S) |
| **titansage02/so101-mcp** ⚠️ | lobehub listing | About 9 tools, mock mode, Claude Desktop config. A pattern reference. |
| **Anthropic Project Fetch** ✅ (phase 1) | red.anthropic.com/2025/project-fetch | Claude helped novices program a robot dog; the biggest uplift was in connecting sensors. Phase 2 "20×" claims appear only in secondary press ⚠️. |
| **GraspGen MCP** ⚠️ | NVlabs/GraspGen release note (Mar 2026) | Relevant only once we have depth. |

## Top 10 to try next

1. Diff i2rt's gravity model and `yam_v1.yml` against DimOS's `yam_gripper_gravity.urdf`; add a per-joint gravity factor in armctl. (S–M)
2. Post our j4 data on i2rt issue #18 and ask on the DimOS Discord. (S)
3. mrcal for wrist-camera intrinsics. (S–M)
4. `calibrateRobotWorldHandEye` plus rotation-diverse `plan_views` (15+ views, all axes, bracket prior). (S)
5. chesscog for the real-board chess layer. (M)
6. An armctl MCP server (patterned on robot_MCP and DimOS MCP). (S–M)
7. LeRobot WebXR phone teleop as an open-play mode. (M)
8. Record a LeRobot-format cup-pick dataset through armctl; train ACT or SmolVLA. (M–L)
9. Fine-tune MolmoAct2-BimanualYAM (or π0.5) for a single arm. (L)
10. Grounded-SAM-2 as a fallback finder. (M)

Parked: GraspGen and GraspGenX (need depth), GELLO (hardware build), lerobot-yam (bypasses the safety layer).

## People, orgs, communities

- **i2rt-robotics** on GitHub: maintainer `i2rt-software`, PR author `nu-jliu`.
- **DimOS:** Discord discord.gg/dimos; AGENTS.md in the repo; hackathons on Luma.
- **Anvil Robotics:** GitHub and HF `anvil-robotics`, CEO Mike Xia. Ask them for the gripper and camera masses and GELLO kit timing.
- **Philipp Wu** (`wuphilipp`, GELLO).
- **`uynitsuj`** (robots_realtime; affiliation not stated on the repo).
- **AI2 MolmoAct team** (`allenai/molmoact2`).
- **Hugging Face LeRobot** Discord and GitHub.
- **Dimios45** (YAM datasets and a GraspGenX fork).
- **Georg Wölflein** (chesscog).
- **Ilia Larchenko** (robot_MCP, lerobot_phone_teleop).
- **Dima Kogan** (`dkogan/mrcal`).

## Not verified

- DimOS licence.
- The lerobot-yam page (did not load).
- Grounded-SAM-2 and GraspGen licences.
- Project Fetch phase 2.
- The ChArUco legacy-pattern note.
- PR #1225 contents.
- Whether any single-arm YAM checkpoint exists.
- Most last-commit dates.
