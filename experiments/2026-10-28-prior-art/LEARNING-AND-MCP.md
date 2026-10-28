# Learned policies on the lab GPU, a data pipeline through armctl, safe deployment, and an armctl MCP server

- **Experimenters:** Naman (+ Claude research agent)
- **Date:** 2026-10-28, ~16:50 IST
- **Context:** the lab laptop has an **NVIDIA RTX 3060 Laptop GPU, 6 GB** (driver 535.183.01); the repo venv has CPU-only torch 2.14.1 (`nvidia-smi`, checked in session).
- **Status:** desk research only (no installs, no hardware). [V] = read at the source, [U] = unverified/estimated. Text by the research agent, saved unchanged by the main session.

## 1. What fits on a 6 GB RTX 3060 Laptop
Use a separate CUDA venv for learning work so the DimOS venv stays as it is.

| Policy | Run on 6 GB? | Train on 6 GB? | Numbers | Cloud fine-tune |
|---|---|---|---|---|
| **LeRobot ACT** (~52M) | Yes | **Yes** | Peak ~2–6 GB at batch 8 (LeRobot guide names the 3060 as a starter GPU) [V]; a user measured ~2 GB on a 3060 12 GB [V]; ~100 ms/inference on a 2021 MacBook [V] | Not needed (HF Jobs t4-small if wanted) |
| **Diffusion Policy** | Yes with DDIM ~10 steps [U] | Marginal (8–14 GB at batch 8 [V]; batch 2–4 + grad accumulation might fit [U]) | | L4/A10G 24 GB, ~2–4 h on a 4090-class card [V], ~$1–3 [U] |
| **SmolVLA** (~450M) | Probably (~1 GB bf16 weights, ~2–3 GB peak) [U] | No (10–16 GB at batch 8 [V]); keep `freeze_vision_encoder=True` | | A100 40 GB batch 16, ~1–2 h [V], ~$2–4 [U] |
| **π0 / π0.5 (openpi)** (~3B) | **No** (>8 GB) [V] | No (LoRA >22.5 GB, full >70 GB [V]; LeRobot 24–40 GB [V]) | No official int8/int4 path [U] | A100 batch 4 ~4–8 h [V], ~$6–12 [V/U]; π0.5 on 4×H100: 3 h 41 min [V] |
| **MolmoAct2** (~5B, BimanualYAM checkpoint) | **No** (12.1 GiB at batch 1 with CUDA graphs) [V] | No (full fine-tune peak 55 GiB [V]); LoRA recommended under 200 demos, global batch 16–32 [V] | | 1× H100 80 GB LoRA ~8–24 h, ~$25–70 [U]; H100 ~$2–3/h, A100 80 GB ~$1.2–1.5/h [V] |
| **Octo** (27M/93M, JAX) | Yes [U] | Small batch possibly [U] | Full fine-tune said to fit 24 GB [U] | One A10G [U]; stale, low priority |
| **RDT-1B** | Unlikely (huge T5-XXL encoder; embeddings can be precomputed) [U] | No | RDT-170M is the low-VRAM option [V] | Multi-A100 [U]; bimanual-oriented, skip |

**Remote inference:** big VLAs on a rented GPU via LeRobot's async PolicyServer/RobotClient (gRPC); SmolVLA on a 4090 over LAN <100 ms round trip [V]. The laptop stays the armctl client.

## 2. Minimal data pipeline (recording through armctl)
- **Gap [V, repo]:** the daemon broadcasts `state` at `STATE_HZ = 20.0` (`armctl/daemon.py:51`); yam-pick-duster-200 uses 25 Hz, LeRobot defaults to 30 fps. Raise to 30 Hz (S) or record at 20 Hz consistently.
- **Per frame:** `observation.state` = q[6] + grip (the `state` message also has qd, torque); `observation.images.wrist` / `.side` (640×480 or smaller, AV1/H.264); `action` = **absolute joint target** q*[6] + grip target (ACT/SmolVLA/π0 default; MolmoAct2 has an absolute-joint mode [V]). Teleop goes through `servo.py` (target → `stream qd`): log the servo *target* (else the next measured q).
- **Episodes:** reset to `ready` → record → end on key/button → mark success/failure; 10–30 s each, each with a task string.
- **How many demos (cup pick):** LeRobot default 50 (~10 per location) [V]; prior-art: 0.93–0.98 success with 180 SO-100 episodes, a 20-demo pick failed [V]; anecdote: 10% at 50, 15–20% at 100 [V]; ACT paper 50 per task [U]. **Plan 50 → 100–200**, training ACT every 25 episodes.
- **Teleop:** phone sticks/tilt in openplay (jerky, 2–3 axes; usable, poor for demos; S) · LeRobot phone 6-DoF WebXR as an openplay mode (good; M) · YAM leader arm, GELLO-style (~$3.2–3.4k [V]; its i2rt script drives the follower on can0 directly → would need porting to feed armctl `stream`; L).
- **Recorder API sketch** (`interfaces/record/recorder.py`): `Recorder(client, cams, repo_id, fps=30, task)`; `start_episode()` (pose≈ready, lease held by the teleop program), `on_action(q_target, grip_target)`, `stop_episode(success)`, `discard_episode()`, `finalize(push=False)` (LeRobotDataset create/add_frame/save_episode). The recorder is a **read-only** armctl program (hello, never acquire; subscribes to `state`; action from the teleop program in-process); drops and logs frames whose camera/state timestamps differ by >1/fps.

## 3. Deploying a policy safely
`camera+state → policy (laptop GPU) → chunk of q* → smoother → servo.py → stream qd (ttl 200 ms) → daemon gate`
- **Rate:** servo at 30 Hz. ACT predicts 100-step chunks: temporal ensembling, or execute ~20–50 steps and re-infer. SmolVLA async with `chunk_size_threshold` ~0.5–0.7, blend favouring the newest chunk [V].
- **Smoothing:** limit target vel/acc below the envelope caps before converting to qd; reject a chunk whose first action is >~0.1 rad from the current q (catches normalisation bugs).
- **Gate/deadman:** safety stays in the daemon. The envelope slides along limits while streaming (no refusal), so a policy can sit pressed against a limit: **detect `refused {stream:true}`** and abort after N. If inference stalls past the 200 ms ttl the backend decelerates: correct; never raise the ttl to cover slow inference.
- **Failures:** stale camera frame → stop streaming · NaN/out-of-range action → `do stop` · lease lost / E-STOP → exit · episode timeout → `stop`, then `goto ready` (operator-approved). First rollouts against mock and DimOS sim (`ZENOH_SCOUT_ADDR=224.0.0.124:7450`), plus open-loop replay of dataset actions.

## 4. armctl MCP server
- **Prior art:** robot_MCP [V]: 4 tools (`get_initial_instructions`, `get_robot_state` with JPEGs of every camera, `move_robot` mm/deg deltas, `control_gripper` %); waits 1 s before the after-move image; **no safety layer**. DimOS MCP [U]: `@skill` methods as tools (`dimos mcp list-tools/call`); would bypass armctl's gate.
- **Proposed tools** (the server is a normal armctl program; the daemon keeps lease, envelope, E-STOP, playable confirmation, execute-then-verify, refusal-not-clamp):

| Tool | armctl ops | Notes |
|---|---|---|
| `observe(cameras=["wrist"], max_px=512)` | `state` + frame grab | q, grip, tip xyz (`kin.tip`), lease/estop/busy, JPEGs at ~512 px (token cost) |
| `get_camera_frame(name, max_px)` | none | single image |
| `request_control()` / `release()` | `acquire` / `release` | operator approves in the console |
| `move_joints(q, speed≤cap)` | `do move_joints` | returns `done` or `refused` + reason |
| `move_to_pose(xyz, down=True)` | `kin.ik` → `do move_joints` | IK in the MCP server; the daemon re-validates |
| `nudge(joint, delta≤0.2)` / `move_tip(dx,dy,dz ≤ 3 cm)` | `do nudge` / IK + `move_joints` | small bounded steps suit LLM latency |
| `goto(name)`, `list_poses()`, `save_pose(name)` | `do goto` / `hello.poses` / `do save_pose` | |
| `gripper(width \| open \| close)` | `do gripper` | |
| `list_playables()`, `play(name)` | `playable.validate` + `playable.play` | a human presses Play |
| `stop()` | `do stop` | works without the lease |

Not exposed: `stream` (LLM latency ≫ 500 ms ttl), `relax`, enable, E-STOP reset, lease override. Every motion tool returns the after-move image and refusal reasons; per-session move budget.

## Recommended order
| # | Work | Effort |
|---|---|---|
| 1 | CUDA torch venv; LeRobot install; state to 30 Hz (or decide on 20) | S |
| 2 | armctl MCP server (tools above) on the mock, then sim | S–M |
| 3 | Recorder + openplay teleop → 10-episode sim dataset, round-trip load | M |
| 4 | Policy runner (chunk → smoother → servo) replaying dataset actions in sim | M |
| 5 | 50–100 real cup-pick demos (phone 6-DoF, or a leader arm) → ACT on the laptop | M–L |
| 6 | SmolVLA fine-tune on a rented A100 (~$5) | M |
| 7 | MolmoAct2 LoRA from BimanualYAM on an H100 with remote async inference | L |

**Sources:** huggingface.co/docs/lerobot/main/hardware_guide.md · github.com/Physical-Intelligence/openpi · huggingface.co/docs/lerobot/main/molmoact2.md · huggingface.co/blog/async-robot-inference · huggingface.co/blog/smolvla · arxiv.org/abs/2506.01844 · wikidocs.net/431677 · huggingface.co/docs/lerobot/il_robots · hackmd.io/@TWFXfRrlSu-lyqTE1BYs5Q/r1GYieZpye · huggingface.co/robotics-diffusion-transformer/rdt-1b · roboticscenter.ai/vla-models/compare/openvla-vs-octo · runpod.io/gpu-cloud/pricing · github.com/pairlab/i2rt · roboticscenter.ai/store/product/i2rt-yam-leader · github.com/IliaLarchenko/robot_MCP · awesome.ecosyste.ms/projects/github.com%2FdimensionalOS%2Fdimos · repo: armctl/README.md, armctl/daemon.py, armctl/kin.py, interfaces/client.py, interfaces/README.md, experiments/2026-10-28-prior-art/REPORT.md
