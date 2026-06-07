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
  // initial guess: direction between centres
  // -------------------------------------------------
  let d = C2.clone().sub(C1).normalize();

  // -------------------------------------------------
  // project helper (same idea as before)
  // -------------------------------------------------
  function projectToPlane(v, axis) {
    const h = v.dot(axis);
    return v.clone().sub(axis.clone().multiplyScalar(h));
  }

  // -------------------------------------------------
  // compute tangent point on a circle for given dir
  // -------------------------------------------------
  function tangentPoint(C, R, axis, dir) {

    // direction perpendicular to radius
    const p = projectToPlane(dir, axis).normalize();

    // perpendicular in plane
    const q = new THREE.Vector3().crossVectors(axis, p).normalize();

    // choose one branch (you’ll later take ±)
    const r = p.clone().multiplyScalar(R);

    return C.clone().add(r);
  }

  // -------------------------------------------------
  // iterative solve
  // -------------------------------------------------
  for (let i = 0; i < 20; i++) {

    const T1 = tangentPoint(C1, R1, axis1, d);
    const T2 = tangentPoint(C2, R2, axis2, d);

    const v = T2.clone().sub(T1);

    // error: v should align with d
    const v_proj = d.clone().multiplyScalar(v.dot(d));
    const err = v.clone().sub(v_proj);

    const errMag = err.length();

    if (errMag < 1e-6) {
      return { T1, T2, dir: d.clone() };
    }

    // nudge direction slightly
    const correction = err.clone().multiplyScalar(-0.5);

    d.add(correction).normalize();
  }

  // fallback (last iteration)
  const T1 = tangentPoint(C1, R1, axis1, d);
  const T2 = tangentPoint(C2, R2, axis2, d);

//  return { T1, T2, dir: d.clone() };
  return [{ T1, T2, dir: d.clone() }];
}
