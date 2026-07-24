import * as THREE from 'three';

function crossSign(a, b, axis) {
  return new THREE.Vector3().crossVectors(a, b).dot(axis);
}

export function tangentsPointCircle(P, C, axis, R) {

  // =====================================================
  // ✅ Step 1: Project point onto circle plane
  // =====================================================
  const v = P.clone().sub(C);
  const h = v.dot(axis);

  const vp = v.clone().sub(
    axis.clone().multiplyScalar(h)
  );

  const d = vp.length();

  if (d <= R) {
    throw new Error("Point is inside or on the circle — no tangents");
  }

  // =====================================================
  // ✅ Step 2: Construct planar basis
  // =====================================================
  const e1 = vp.clone().normalize(); // now guaranteed in plane
  const e2 = new THREE.Vector3()
    .crossVectors(axis, e1)
    .normalize(); // also in plane

  // =====================================================
  // ✅ Step 3: Compute tangent directions
  // =====================================================
  const cosA = R / d;
  const sinA = Math.sqrt(1 - cosA * cosA);

  const dir1 = e1.clone().multiplyScalar(cosA)
    .addScaledVector(e2,  sinA);

  const dir2 = e1.clone().multiplyScalar(cosA)
    .addScaledVector(e2, -sinA);

  // =====================================================
  // ✅ Step 4: Compute tangent points
  // =====================================================
  const T1 = C.clone().addScaledVector(dir1, R);
  const T2 = C.clone().addScaledVector(dir2, R);

  // =====================================================
  // ✅ Return results
  // =====================================================
  return [
    { T: T1 },
    { T: T2 }
  ];
}

export function tangentsCircleCircle(C1, R1, C2, R2, axis) {

  const results = [];

  // =====================================================
  // ✅ Step 1: project C2 into plane of C1
  // =====================================================
  const v = C2.clone().sub(C1);
  const h = v.dot(axis);

  const C2p = C2.clone().sub(
    axis.clone().multiplyScalar(h)
  );

  const vp = C2p.clone().sub(C1);
  const d = vp.length();

  if (d < 1e-8) return results;

  const e1 = vp.clone().normalize();
  const e2 = new THREE.Vector3()
    .crossVectors(axis, e1)
    .normalize();

  // =====================================================
  // ✅ helper
  // =====================================================
  function build(R_eff, sign) {

    const cosA = R_eff / d;

    if (Math.abs(cosA) > 1) return;

    const sinA = Math.sqrt(1 - cosA * cosA);

    const dirs = [
      e1.clone().multiplyScalar(cosA)
        .addScaledVector(e2,  sinA),

      e1.clone().multiplyScalar(cosA)
        .addScaledVector(e2, -sinA)
    ];

    for (const dir of dirs) {

      const T1 = C1.clone().addScaledVector(dir, R1);

      const T2 = C2.clone().addScaledVector(
        dir,
        sign * R2
      );

      results.push({ T1, T2 });
    }
  }

  // external
  build(R2 - R1, +1);

  // internal
  build(R2 + R1, -1);

  return results;
}

export function isConsistent(P_prev, T_in, T_out, P_next, C, axis) {
  const vin = T_in.clone().sub(P_prev).normalize();
  const rin = T_in.clone().sub(C).normalize();

  const vout = P_next.clone().sub(T_out).normalize();
  const rout = T_out.clone().sub(C).normalize();

  const s1 = crossSign(vin, rin, axis);
  const s2 = crossSign(vout, rout, axis);

  return s1 * s2 > 0;
}

export function arcAngle(C, axis, T1, T2) {
  const r1 = T1.clone().sub(C).normalize();
  const r2 = T2.clone().sub(C).normalize();

  const cross = new THREE.Vector3().crossVectors(r1, r2);
  const dot = r1.dot(r2);

  return Math.abs(Math.atan2(cross.dot(axis), dot));
}

export function addArc(segments, C, axis, R, T1, T2, N = 24) {
  const ax = axis.clone().normalize();

  const r1 = T1.clone().sub(C).normalize();
  const r2 = T2.clone().sub(C).normalize();

  const e1 = r1;
  const e2 = new THREE.Vector3().crossVectors(ax, e1).normalize();

  const a2 = Math.atan2(r2.dot(e2), r2.dot(e1));

  let delta = a2;
  if (delta > Math.PI) delta -= 2 * Math.PI;
  if (delta < -Math.PI) delta += 2 * Math.PI;

  let prev = T1.clone();

  for (let i = 1; i <= N; i++) {
    const pt = (i === N)
      ? T2.clone()
      : C.clone()
          .addScaledVector(e1, R * Math.cos(i / N * delta))
          .addScaledVector(e2, R * Math.sin(i / N * delta));

    segments.push({ type: 'line', from: prev, to: pt });
    prev = pt;
  }
}



export class CircularArc3D extends THREE.Curve {

  constructor(C, axis, R, T1, T2, useLongArc = false) {
    super();

    this.C = C.clone();
    this.axis = axis.clone().normalize();
    this.R = R;

    this.r1 = T1.clone().sub(C).normalize();
    this.r2 = T2.clone().sub(C).normalize();

    this.e1 = this.r1;
    this.e2 = new THREE.Vector3()
      .crossVectors(this.axis, this.e1)
      .normalize();

//    this.e1 = this.r1;

//    // build e2 directly from r2
//    const r2_proj = this.r2.clone().sub(
//      this.e1.clone().multiplyScalar(this.r2.dot(this.e1))
//    );

//    this.e2 = r2_proj.normalize();


//    let angle = Math.atan2(
//      this.r2.dot(this.e2),
//      this.r2.dot(this.e1)
//    );


    const cross = new THREE.Vector3().crossVectors(this.r1, this.r2);
    const dot   = this.r1.dot(this.r2);

    let angle = Math.atan2(
      cross.dot(this.axis),   // signed magnitude
      dot
    );

//    this.useLongArc = useLongArc;
    // ✅ only this extra rule
    if (useLongArc) {
      if (angle > 0) angle -= 2 * Math.PI;
      else angle += 2 * Math.PI;
    }

    this.angle = angle;
  }

  getPoint(t) {
    const a = t * this.angle;

    return this.C.clone()
      .addScaledVector(this.e1, this.R * Math.cos(a))
      .addScaledVector(this.e2, this.R * Math.sin(a));
  }

//  getPoint(t) {

//    const v = this.r1.clone().multiplyScalar(1 - t)
//      .addScaledVector(this.r2, t)
//      .normalize();

//    return this.C.clone().addScaledVector(v, this.R);
//  }
//  getPoint(t) {

//    const cross = new THREE.Vector3().crossVectors(this.r1, this.r2);
//    const dot = this.r1.dot(this.r2);

//    // short arc angle
//    let theta = Math.atan2(cross.dot(this.axis), dot);

//    // enforce branch
//    if (this.useLongArc) {
//      if (theta > 0) theta -= 2 * Math.PI;
//      else           theta += 2 * Math.PI;
//    }

//    const angle = t * theta;

//    const v = this.r1.clone().multiplyScalar(Math.cos(angle))
//      .addScaledVector(
//        new THREE.Vector3().crossVectors(this.axis, this.r1),
//        Math.sin(angle)
//      );

//    return this.C.clone().addScaledVector(v, this.R);
//  }
}


export function appendCurve(segments, curve, N = 32) {
  const pts = curve.getPoints(N);

  for (let i = 1; i < pts.length; i++) {
    segments.push({
      type: 'line',
      from: pts[i - 1],
      to: pts[i]
    });
  }
}


export function tangentCircleCircle3D(C1, R1, axis1, C2, R2, axis2) {

  // -------------------------------------------------
  // Initial guess: direction between centres
  // -------------------------------------------------
  let d = C2.clone().sub(C1).normalize();

  // -------------------------------------------------
  // Construct tangent points for a given direction
  // Ensures:
  //   - point lies in circle plane
  //   - radius ⟂ direction (true tangency)
  // -------------------------------------------------
function tangentPoints(C, R, axis, dir) {

  // STEP 1: project dir into plane
  const d_proj = dir.clone().sub(
    axis.clone().multiplyScalar(dir.dot(axis))
  );

  if (d_proj.lengthSq() < 1e-12) return [];

  d_proj.normalize();

  // STEP 2: rotate 90° in plane
  const rDir = new THREE.Vector3()
    .crossVectors(axis, d_proj)
    .normalize();

  return [
    C.clone().addScaledVector(rDir,  R),
    C.clone().addScaledVector(rDir, -R)
  ];
}
  // -------------------------------------------------
  // Iterative solve for direction
  // -------------------------------------------------
  for (let i = 0; i < 20; i++) {

    // use consistent branch during iteration
    const [T1] = tangentPoints(C1, R1, axis1, d);
    const [T2] = tangentPoints(C2, R2, axis2, d);

    if (!T1 || !T2) break;

    const v = T2.clone().sub(T1);

    // component of v perpendicular to d
    const v_proj = d.clone().multiplyScalar(v.dot(d));
    const err = v.clone().sub(v_proj);

    if (err.length() < 1e-6) break;

    // update direction
    d.add(err.clone().multiplyScalar(-0.5)).normalize();
  }


  const [T1aa] = tangentPoints(C1, R1, axis1, d);
  const [T2aa] = tangentPoints(C2, R2, axis2, d);

  const r1 = T1aa.clone().sub(C1);
  const r2 = T2aa.clone().sub(C2);

  console.log("check1 (should be 0):", r1.dot(d));
  console.log("check2 (should be 0):", r2.dot(d));






  // -------------------------------------------------
  // Build final solutions (± branch)
  // -------------------------------------------------
  const pts1 = tangentPoints(C1, R1, axis1, d);
  const pts2 = tangentPoints(C2, R2, axis2, d);

  // guard against degeneracy
  if (pts1.length < 2 || pts2.length < 2) {
    return [];
  }

  const [T1a, T1b] = pts1;
  const [T2a, T2b] = pts2;



  const v = T2a.clone().sub(T1a).normalize();

  console.log("line alignment:", v.dot(d));

  return [
    { T1: T1a, T2: T2a, dir: d.clone() },
    { T1: T1b, T2: T2b, dir: d.clone() }
  ];
}

function solveWithInitial(C1, R1, axis1, C2, R2, axis2, d0) {

  let d = d0.clone().normalize();

  function tangentPoints(C, R, axis, dir) {
    const d_proj = dir.clone().sub(
      axis.clone().multiplyScalar(dir.dot(axis))
    );

    if (d_proj.lengthSq() < 1e-12) return [];

    d_proj.normalize();

    const rDir = new THREE.Vector3()
      .crossVectors(axis, d_proj)
      .normalize();

    return [
      C.clone().addScaledVector(rDir,  R),
      C.clone().addScaledVector(rDir, -R)
    ];
  }

  // --- iterate ---
  for (let i = 0; i < 20; i++) {

    const [T1] = tangentPoints(C1, R1, axis1, d);
    const [T2] = tangentPoints(C2, R2, axis2, d);

    if (!T1 || !T2) break;

    const r1 = T1.clone().sub(C1);
    const r2 = T2.clone().sub(C2);

    const err1 = r1.dot(d);
    const err2 = r2.dot(d);

    if (Math.abs(err1) + Math.abs(err2) < 1e-6) break;

    const grad = r1.clone().multiplyScalar(err1)
      .addScaledVector(r2, err2);

    d.sub(grad.multiplyScalar(0.2)).normalize();
  }

  // --- final points ---
  const pts1 = tangentPoints(C1, R1, axis1, d);
  const pts2 = tangentPoints(C2, R2, axis2, d);

  if (pts1.length < 2 || pts2.length < 2) return [];

  const [T1a, T1b] = pts1;
  const [T2a, T2b] = pts2;

  return [
    { T1: T1a, T2: T2a, dir: d.clone() },
    { T1: T1b, T2: T2b, dir: d.clone() }
  ];
}

export function tangentCircleCircle3D_All(C1, R1, axis1, C2, R2, axis2) {

  const results = [];

  // =====================================================
  // Tangent construction (correct, verified)
  // =====================================================
  function tangentPoints(C, R, axis, dir) {

    const d_proj = dir.clone().sub(
      axis.clone().multiplyScalar(dir.dot(axis))
    );

    if (d_proj.lengthSq() < 1e-12) return [];

    d_proj.normalize();

    const rDir = new THREE.Vector3()
      .crossVectors(axis, d_proj)
      .normalize();

    return [
      C.clone().addScaledVector(rDir,  R),
      C.clone().addScaledVector(rDir, -R)
    ];
  }

  // =====================================================
  // Build only VALID solutions (key filter)
  // =====================================================
  function buildSolutions(d, pts1, pts2) {

    if (pts1.length < 2 || pts2.length < 2) return [];

    const [T1a, T1b] = pts1;
    const [T2a, T2b] = pts2;

    const candidates = [
      { T1: T1a, T2: T2a },
      { T1: T1b, T2: T2b }
    ];

    const out = [];

    for (const c of candidates) {

      const v = c.T2.clone().sub(c.T1).normalize();

      // ✅ keep only if aligned with direction
      if (Math.abs(v.dot(d)) > 0.99) {
        out.push({
          T1: c.T1,
          T2: c.T2,
          dir: d.clone()
        });
      }
    }

    return out;
  }

  // =====================================================
  // Single solver (one family)
  // =====================================================
  function solveWithInitial(d0, R2_eff) {

    let d = d0.clone().normalize();

    for (let i = 0; i < 20; i++) {

      const [T1] = tangentPoints(C1, R1, axis1, d);
      const [T2] = tangentPoints(C2, R2_eff, axis2, d);

      if (!T1 || !T2) break;

      const r1 = T1.clone().sub(C1);
      const r2 = T2.clone().sub(C2);

      const err1 = r1.dot(d);
      const err2 = r2.dot(d);

      if (Math.abs(err1) + Math.abs(err2) < 1e-6) break;

      const grad = r1.clone().multiplyScalar(err1)
        .addScaledVector(r2, err2);

      d.sub(grad.multiplyScalar(0.2)).normalize();
    }

    const pts1 = tangentPoints(C1, R1, axis1, d);
    const pts2 = tangentPoints(C2, R2_eff, axis2, d);

    return buildSolutions(d, pts1, pts2);
  }

  // =====================================================
  // Seed generation (robust exploration)
  // =====================================================
  const v = C2.clone().sub(C1).normalize();
  const n = new THREE.Vector3().crossVectors(axis1, v).normalize();

  const seeds = [
    v,
    v.clone().negate(),

    n,
    n.clone().negate(),

    v.clone().add(n).normalize(),
    v.clone().sub(n).normalize(),
    v.clone().negate().add(n).normalize(),
    v.clone().negate().sub(n).normalize()
  ];

  // =====================================================
  // OUTER tangents
  // =====================================================
  for (const d0 of seeds) {
    results.push(...solveWithInitial(d0,  R2));
  }

  // =====================================================
  // INNER tangents (critical: sign flip)
  // =====================================================
  for (const d0 of seeds) {
    results.push(...solveWithInitial(d0, -R2));
  }

  // =====================================================
  // Deduplicate
  // =====================================================
  return uniqueSolutions(results);
}

function buildSolutions(d, pts1, pts2) {

  if (pts1.length < 2 || pts2.length < 2) return [];

  const [T1a, T1b] = pts1;
  const [T2a, T2b] = pts2;

  const candidates = [
    { T1: T1a, T2: T2a },
    { T1: T1b, T2: T2b }
  ];

  const out = [];

  for (const c of candidates) {

    const v = c.T2.clone().sub(c.T1).normalize();

    // ✅ Keep only if aligned with direction
    if (Math.abs(v.dot(d)) > 0.99) {
      out.push({
        T1: c.T1,
        T2: c.T2,
        dir: d.clone()
      });
    }
  }

  return out;
}

function uniqueSolutions(sols) {

  const out = [];

  for (const s of sols) {

    const exists = out.some(o =>
      o.T1.distanceTo(s.T1) < 1e-5 &&
      o.T2.distanceTo(s.T2) < 1e-5
    );

    if (!exists) out.push(s);
  }

  return out;
}

/**
 * Estimate eye circumference based on simplified EN 13414-3 geometry.
 *
 * Model:
 * - Eye approximated as:
 *   • isosceles triangle + semicircle cap
 * - Total height h = 15 d
 * - Cap diameter w = h / 2 = 7.5 d
 *
 * Derived circumference:
 *   C = d * (15 * sqrt(5/2) + (15π)/4)
 *
 * Reference:
 * - EN 13414-3, Section 5.2.2 (geometry inspired by Fig. 2)
 * - This is an approximation for default visualisation only
 *
 * @param {number} d - rope diameter
 * @returns {number} eye circumference
 */
export function estimateEyeCircumference(d) {
  return d * (
    15 * Math.sqrt(5 / 2) +
    (15 * Math.PI) / 4
  );
}

