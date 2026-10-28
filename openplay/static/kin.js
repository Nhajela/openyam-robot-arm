// Forward kinematics of the YAM arm for the open-play display (fallback 3D view and ghost
// previews). A small JS port of armctl/kin.py: the joint origins and axes are copied from
// yam_description/i2rt/yam.urdf (same table as armctl/kin.py CHAIN + TIP_OFFSET).
// Units: metres, radians. Base frame: z up, x forward (where the arm reaches at j1 = 0).
// Known values (pinocchio): home tip (0.211, 0, 0.174), ready tip (0.205, 0, 0.432).
// Nothing here moves anything: it only draws.

// [name, child, xyz, rpy, axis] for joint1..joint6.
export const CHAIN = [
  ['joint1', 'link1', [-2.49777947129965e-7, -9.17402363407408e-13, 0.0679999999995413], [0, 1.5708, 3.1415963267949], [-1, 0, 0]],
  ['joint2', 'link2', [-0.0455, -0.0339, -0.02], [-1.7953e-16, 5.55112e-17, -3.14159], [3.58252e-15, 1, -3.2007e-16]],
  ['joint3', 'link3', [4.08431e-7, -0.0688, 0.264], [-2.99502e-14, -1.6015e-14, -3.14159], [4.54273e-15, 1, 1.04275e-14]],
  ['joint4', 'link4', [-0.0600003, -0.0688, -0.244999], [2.95536e-14, 8.52374e-14, -2.10585e-14], [0, 1, 0]],
  ['joint5', 'link5', [-0.0405003, 0.0338995, -0.0739989], [2.73264e-14, -4.32956e-16, -8.32667e-17], [1, 0, 0]],
  ['joint6', 'gripper', [0.0404996, 2.39858e-7, -0.0356], [8.83309e-16, -5.88588e-16, 7.69861e-14], [1.46014e-15, 2.40808e-14, -1]],
];
export const TIP_OFFSET = [0, 0, -0.1]; // gripper -> gripper_tip (fixed); -z points toward the fingertips
// Fingers (joint7/joint8) slide along the gripper frame's y axis, stroke 0..0.04695 m.
export const FINGER_STROKE = 0.04695;
export const URDF_LIMITS = [[-2.618, 3.142], [0.0, 3.665], [0.0, 3.142], [-1.693, 1.571], [-1.571, 1.571], [-2.094, 2.094]];
export const POSES = { home: [0, 0, 0, 0, 0, 0], ready: [0, 0.5, 0.8, 0, 0, 0] };

// 4x4 transforms are {R: 3x3 row-major nested array, p: [x, y, z]}.
function originT(xyz, rpy) {
  const [r, p, y] = rpy;
  const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y);
  return {
    R: [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]],
    p: xyz.slice(),
  };
}
function axisAngleR(axis, a) {
  const n = Math.hypot(axis[0], axis[1], axis[2]);
  const x = axis[0] / n, y = axis[1] / n, z = axis[2] / n;
  const c = Math.cos(a), s = Math.sin(a), C = 1 - c;
  return [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
          [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
          [z * x * C - y * s, z * y * C + x * s, c + z * z * C]];
}
const mulR = (A, B) => A.map((row) => [0, 1, 2].map((j) => row[0] * B[0][j] + row[1] * B[1][j] + row[2] * B[2][j]));
const mulRv = (A, v) => A.map((row) => row[0] * v[0] + row[1] * v[1] + row[2] * v[2]);
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
// T1 @ T2
export function compose(T1, T2) { return { R: mulR(T1.R, T2.R), p: add(mulRv(T1.R, T2.p), T1.p) }; }
// Point v (in T's frame) -> base frame.
export function apply(T, v) { return add(mulRv(T.R, v), T.p); }

const ORIGINS = CHAIN.map(([, , xyz, rpy]) => originT(xyz, rpy));
const IDENT = () => ({ R: [[1, 0, 0], [0, 1, 0], [0, 0, 1]], p: [0, 0, 0] });

// Pose of every link (and gripper_tip) in the base frame: {base, link1..link5, gripper, gripper_tip}.
export function frames(q) {
  let T = IDENT();
  const out = { base: T };
  CHAIN.forEach(([, child, , , axis], i) => {
    T = compose(compose(T, ORIGINS[i]), { R: axisAngleR(axis, +q[i] || 0), p: [0, 0, 0] });
    out[child] = T;
  });
  out.gripper_tip = compose(T, { R: IDENT().R, p: TIP_OFFSET.slice() });
  return out;
}
export function tip(q) { return frames(q).gripper_tip.p.slice(); }

// ---- keyframe playback (ghost previews) ----
// Playable keyframes: [{t, q: [6], grip?, ease?}], t in seconds from the start, grip 0 closed..1 open
// (carried forward when missing). Between two keyframes the motion follows the later one's ease:
// "linear" is linear, "flow" is armctl.playable's monotone Hermite (passes through a keyframe
// between two flow segments without stopping), anything else ("cubic", missing) is smoothstep.
const smoothstep = (s) => s * s * (3 - 2 * s);
// Same rule as armctl.playable.flow_tangents: PCHIP (Fritsch-Butland) tangent per channel at a
// keyframe between two flow segments (not "stop": true), 0 where the channel turns round.
function flowTangents(out) {
  for (let i = 0; i < out.length; i++) {
    const k = out[i];
    k.m = new Array(7).fill(0);
    if (i === 0 || i === out.length - 1 || k.stop || k.ease !== 'flow' || out[i + 1].ease !== 'flow') continue;
    const a = out[i - 1], b = out[i + 1];
    const h0 = k.t - a.t, h1 = b.t - k.t;
    if (h0 <= 0 || h1 <= 0) continue;
    const va = [...a.q, a.grip], vk = [...k.q, k.grip], vb = [...b.q, b.grip];
    const w1 = 2 * h1 + h0, w2 = h1 + 2 * h0;
    for (let c = 0; c < 7; c++) {
      const d0 = (vk[c] - va[c]) / h0, d1 = (vb[c] - vk[c]) / h1;
      k.m[c] = d0 * d1 > 0 ? (w1 + w2) / (w1 / d0 + w2 / d1) : 0;
    }
  }
  return out;
}
export function normaliseKeyframes(kfs) {
  if (!Array.isArray(kfs)) return [];
  let grip = 0;
  const out = [];
  for (const k of kfs) {
    if (!k || !Array.isArray(k.q) || k.q.length < 6 || !Number.isFinite(+k.t)) continue;
    if (Number.isFinite(+k.grip)) grip = +k.grip;
    out.push({ t: +k.t, q: k.q.slice(0, 6).map(Number), grip, ease: k.ease || 'cubic', stop: k.stop === true });
  }
  out.sort((a, b) => a.t - b.t);
  return flowTangents(out);
}
export function keyframesDuration(kfs) { return kfs.length ? kfs[kfs.length - 1].t : 0; }
// Pose at time t (clamped to the ends). kfs must be normalised.
export function sampleKeyframes(kfs, t) {
  if (!kfs.length) return null;
  if (t <= kfs[0].t) return { q: kfs[0].q.slice(), grip: kfs[0].grip };
  const last = kfs[kfs.length - 1];
  if (t >= last.t) return { q: last.q.slice(), grip: last.grip };
  let i = 0;
  while (i < kfs.length - 2 && kfs[i + 1].t <= t) i++;
  const a = kfs[i], b = kfs[i + 1];
  const span = Math.max(b.t - a.t, 1e-6);
  let s = Math.min(1, Math.max(0, (t - a.t) / span));
  if (b.ease === 'flow' && a.m && b.m) {
    const h10 = s * (1 - s) ** 2 * span, h11 = s * s * (s - 1) * span, w = smoothstep(s);
    return { q: a.q.map((v, j) => v + (b.q[j] - v) * w + h10 * a.m[j] + h11 * b.m[j]),
             grip: a.grip + (b.grip - a.grip) * w + h10 * a.m[6] + h11 * b.m[6] };
  }
  if (b.ease !== 'linear') s = smoothstep(s);
  return { q: a.q.map((v, j) => v + (b.q[j] - v) * s), grip: a.grip + (b.grip - a.grip) * s };
}
