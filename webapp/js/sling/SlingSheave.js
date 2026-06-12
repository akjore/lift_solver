import * as THREE from 'three';

export function buildSheave(P_in, C, P_out, D, ropeRadius) {
  const EPS = 1e-6;

  const R = D / 2 + ropeRadius;

  const segments = [];

  function safePush(a, b, radius) {
    if (!a || !b) return;

    if (
      !isFinite(a.x) || !isFinite(a.y) || !isFinite(a.z) ||
      !isFinite(b.x) || !isFinite(b.y) || !isFinite(b.z)
    ) return;

    if (a.distanceTo(b) < EPS) return;

    segments.push({
      type: 'line',
      from: a,
      to: b,
      radius
    });
  }

  // =========================================================
  // ✅ Directions into and out of sheave
  // =========================================================

  const dirIn  = P_in.clone().sub(C).normalize();
  const dirOut = P_out.clone().sub(C).normalize();

  // =========================================================
  // ✅ Local plane
  // =========================================================

  let planeNormal = new THREE.Vector3().crossVectors(dirIn, dirOut);

  if (planeNormal.length() < EPS) {
    planeNormal = new THREE.Vector3(0, 0, 1).cross(dirIn);

    if (planeNormal.length() < EPS) {
      planeNormal = new THREE.Vector3(0, 1, 0).cross(dirIn);
    }
  }

  planeNormal.normalize();

  const e1 = dirIn.clone();
  const e2 = new THREE.Vector3()
    .crossVectors(planeNormal, e1)
    .normalize();

  // =========================================================
  // ✅ Angles on sheave circle
  // =========================================================

  function angleOf(v) {
    return Math.atan2(v.dot(e2), v.dot(e1));
  }

  const a_in  = angleOf(dirIn);
  let   a_out = angleOf(dirOut);

  // ✅ enforce shortest arc
  let delta = a_out - a_in;

  if (delta > Math.PI) delta -= 2 * Math.PI;
  if (delta < -Math.PI) delta += 2 * Math.PI;

//  delta = 0;
  if (delta > 0) {
    delta -= 2 * Math.PI;
  } else {
    delta += 2 * Math.PI;
  }

  //  delta = -delta;
//  delta = Math.PI - delta;

  // =========================================================
  // ✅ Tangent points on sheave
  // =========================================================

  const T_in = C.clone()
    .addScaledVector(e1, R * Math.cos(a_in))
    .addScaledVector(e2, R * Math.sin(a_in));

  const T_out = C.clone()
    .addScaledVector(e1, R * Math.cos(a_in + delta))
    .addScaledVector(e2, R * Math.sin(a_in + delta));

  // =========================================================
  // ✅ Build segments
  // =========================================================

  // incoming straight
  safePush(P_in, T_in, 'nominal');

  // arc
  const N = 32;
  let prev = T_in;

  for (let i = 1; i <= N; i++) {

    const t = i / N;
    const angle = a_in + t * delta;

    const pt = C.clone()
      .addScaledVector(e1, R * Math.cos(angle))
      .addScaledVector(e2, R * Math.sin(angle));

    safePush(prev, pt, 'nominal');
    prev = pt;
  }

  // outgoing straight
  safePush(T_out, P_out, 'nominal');

  return {
    segments,
    T_in,
    T_out
  };
}
