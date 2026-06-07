import * as THREE from 'three';
import { CircularArc3D, appendCurve } from './GeometryUtils.js';

function solveAlpha(l_eye, R) {
  // alpha represents half the short arc between the two tangents

  function f(x) {
    return Math.tan(x) - x - l_eye / R + Math.PI;
  }

  function df(x) {
    const cos = Math.cos(x);
    return 1 / (cos * cos) - 1; // sec²(x) - 1
  }

  let x = Math.PI / 2 * 0.99;

  for (let i = 0; i < 20; i++) {

    const fx = f(x);
    const dfx = df(x);

    if (!isFinite(fx) || !isFinite(dfx)) {
      throw new Error("Newton failed in eye solver");
    }

    if (Math.abs(dfx) < 1e-10) break;

    const step = fx / dfx;
    x = x - step;

    if (Math.abs(step) < 1e-10) break;
  }

  return x;
}

export class SlingEye {

  constructor(params) {
    this.C = new THREE.Vector3(...params.center);
    this.axis = new THREE.Vector3(...params.axis).normalize();
    this.direction = new THREE.Vector3(...params.direction).normalize();

    this.d = params.d;   // rope diameter
    this.D = params.D;   // ✅ pin diameter
    this.eyeCircumference = params.eyeCircumference;
    this.L_splice = params.L_splice;
  }

  build() {

    const segments = [];

    const C = this.C.clone();
    const axis = this.axis.clone();
    const dir = this.direction.clone();

    const ropeRadius = this.d / 2;

    // ✅ Correct effective radius (wraps around pin)
    const R = this.D / 2 + ropeRadius;

    // =====================================================
    // ✅ Solve alpha (your original formulation)
    // =====================================================
    const alpha = solveAlpha(this.eyeCircumference/2, R);

    // =====================================================
    // ✅ Splice geometry (on centreline)
    // =====================================================
    const d = R / Math.cos(alpha);
    const spliceEnd = C.clone().addScaledVector(dir, d);

    const spliceStart = spliceEnd.clone().addScaledVector(dir, this.L_splice);

    // splice segment
    segments.push({
      type: 'line',
      from: spliceEnd.clone(),
      to: spliceStart.clone(),
      radius: 'splice'
    });


    // =====================================================
    // ✅ Build tangent points
    // =====================================================
//    const v = spliceEnd.clone().sub(C);
//    const e1 = v.clone().normalize();

    const v = spliceEnd.clone().sub(C);

    // project into sheave plane
    const h = v.dot(axis);

    const v_proj = v.clone().sub(
      axis.clone().multiplyScalar(h)
    );

    const e1 = v_proj.normalize()



    const e2 = new THREE.Vector3().crossVectors(axis, e1).normalize();

    const r1 = e1.clone().multiplyScalar(Math.cos(alpha))
      .addScaledVector(e2, Math.sin(alpha));

    const r2 = e1.clone().multiplyScalar(Math.cos(alpha))
      .addScaledVector(e2, -Math.sin(alpha));

    const T1 = C.clone().addScaledVector(r1, R);
    const T2 = C.clone().addScaledVector(r2, R);


    console.log("T1 plane error:", (T1.clone().sub(C)).dot(axis));
    console.log("T2 plane error:", (T2.clone().sub(C)).dot(axis));

    // =====================================================
    // ✅ Straight legs (spliceEnd → tangents)
    // =====================================================
    segments.push({
      type: 'line',
      from: spliceEnd.clone(),
      to: T1.clone(),
      radius: 'nominal'
    });

    segments.push({
      type: 'line',
      from: spliceEnd.clone(),
      to: T2.clone(),
      radius: 'nominal'
    });


    // =====================================================
    // ✅ Arc between tangents (LONG wrap)
    // =====================================================
    const eA = r1;
    const eB = new THREE.Vector3().crossVectors(axis, eA).normalize();

    const a1 = 0;
    const a2 = Math.atan2(
      r2.dot(eB),
      r2.dot(eA)
    );

    let delta = a2;

    // force long arc (> π)
    if (delta > 0) {
      delta = delta - 2 * Math.PI;
    } else {
      delta = delta + 2 * Math.PI;
    }


    const curve = new CircularArc3D(C, axis, R, T1, T2, true);

    appendCurve(segments, curve, 32);

    // 🔴 IMPORTANT CHECK
//    const p0 = curve.getPoint(0);
//    const p1 = curve.getPoint(1);

//    console.log("p0 vs T1:", p0.distanceTo(T1));
//    console.log("p1 vs T2:", p1.distanceTo(T2));



    const pts = curve.getPoints(32);

    console.log("T1 error:", pts[0].distanceTo(T1));
    console.log("T2 error:", pts[pts.length-1].distanceTo(T2));

    console.log("T1 radius error:", T1.distanceTo(C) - R);
    console.log("T2 radius error:", T2.distanceTo(C) - R)


//    const N = 32;
//    let prev = T1.clone();

//    for (let i = 1; i <= N; i++) {

//      const t = i / N;
//      const angle = a1 + t * delta;

//      const pt = (i === N)
//        ? T2.clone()
//        : C.clone()
//            .addScaledVector(eA, R * Math.cos(angle))
//            .addScaledVector(eB, R * Math.sin(angle));

//      segments.push({
//        type: 'line',
//        from: prev.clone(),
//        to: pt.clone(),
//        radius: 'nominal'
//      });

//      prev = pt;
//    }


    // =====================================================
    // ✅ Return result
    // =====================================================
    return {
      segments,
      spliceStart,
      spliceEnd
    };
  }
}