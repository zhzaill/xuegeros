# Included dependencies

Three.js r128 (0.128.0), including OrbitControls and STLLoader, is distributed
under the MIT License. Copyright © 2010–2021 Three.js authors.
Full upstream license: `runtime/mcp-xuegeros/xuegeros_mcp/vendor/three/LICENSE`.
Source: https://github.com/mrdoob/three.js/tree/r128

The npm package contains project Python sources, two project SDF scenes,
a URDF description, and a procedural vehicle renderer. CAD STL meshes,
YOLO weights, ROS installations, logs and credentials are excluded.
External meshes may be loaded with `--mesh-dir`; their owner supplies the
appropriate rights. No license for external meshes is granted by this package.

Python 3.10+ is an external runtime. Pillow is an optional external dependency
used by map PNG rendering; it is neither bundled nor installed automatically.
