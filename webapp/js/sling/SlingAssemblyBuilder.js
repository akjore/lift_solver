import * as THREE from 'three';
import { SlingEye } from './SlingEye.js';

export class SlingAssemblyBuilder {

  constructor(data) {
    this.data = data;
  }

  build() {

    const segments = [];
    const ropeRadius = this.data.d / 2;

    const startPin = this.data.startPin;
    const endPin   = this.data.endPin;
    const sheaves  = this.data.sheaves || [];

    // =====================================================
    // ✅ START EYE
    // =====================================================
    const firstTarget = sheaves.length > 0
      ? new THREE.Vector3(...sheaves[0].center)
      : new THREE.Vector3(...endPin.center);

    const startDir = firstTarget.clone()
      .sub(new THREE.Vector3(...startPin.center))
      .normalize();

    const startEye = new SlingEye({
      ...startPin,
      D: startPin.D,             // ✅ ADD THIS
      d: this.data.d,
      eyeCircumference: this.data.eyeCircumference,
      L_splice: this.data.spliceLength,
      direction: startDir.toArray()
    }).build();

    segments.push(...startEye.segments);

    let currentPoint = startEye.spliceStart.clone();


    // =====================================================
    // ✅ THROUGH SHEAVES (centre-line only)
    // =====================================================
    for (let i = 0; i < sheaves.length; i++) {

      const C = new THREE.Vector3(...sheaves[i].center);

      segments.push({
        type: 'line',
        from: currentPoint.clone(),
        to: C.clone(),
        radius: 'nominal'
      });

      currentPoint = C.clone();
    }


    // =====================================================
    // ✅ END EYE
    // =====================================================
    const endCenter = new THREE.Vector3(...endPin.center);

    const endDir = endCenter.clone()
      .sub(currentPoint)
      .normalize()
      .multiplyScalar(-1);

    const endEye = new SlingEye({
      ...endPin,
      D: endPin.D,               // ✅ ADD THIS
      d: this.data.d,
      eyeCircumference: this.data.eyeCircumference,
      L_splice: this.data.spliceLength,
      direction: endDir.toArray()
    }).build();

    // connect rope → splice
    segments.push({
      type: 'line',
      from: currentPoint.clone(),
      to: endEye.spliceEnd.clone(),
      radius: 'nominal'
    });

    segments.push(...endEye.segments);


    // =====================================================
    // ✅ OUTPUT
    // =====================================================
    return {
      segments,
      radii: {
        nominal: ropeRadius,
        splice: Math.sqrt(2) * ropeRadius
      }
    };
  }
}