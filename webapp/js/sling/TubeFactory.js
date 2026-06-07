import * as THREE from 'three';

export class TubeFactory {
  static buildSegments(segments, radii) {
    const group = new THREE.Group();

    segments.forEach(seg => {
      const radius = radii[seg.radius || 'nominal'];
      if (!seg.from || !seg.to) return;

      const path = new THREE.LineCurve3(seg.from, seg.to);
      const geo = new THREE.TubeGeometry(path, 20, radius, 12, false);
      const mat = new THREE.MeshStandardMaterial({ color: 0x0044cc });

      group.add(new THREE.Mesh(geo, mat));
    });

    return group;
  }
}
