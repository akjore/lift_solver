# lift_solver

A YAML-driven multibody lifting and rigging solver built on
[**Exudyn**](https://github.com/jgerstmayr/EXUDYN),
an open-source multibody dynamics simulation framework.

---

## 🚀 Overview

This project provides a clean, solver-agnostic workflow for defining and solving lifting problems:

- Rigid bodies (loads, spreaders, hooks)
- Shackles and connection hardware
- Slings (elastic elements)
- Realistic joint behaviour (pins, fixed connections)

---

## 🧠 Design Philosophy

The solver follows a strict layered approach:

YAML → What is connected
Middleware → Geometry and semantics
Solver layer → Numerical implementation (Exudyn)

The user never needs to understand solver internals.

---

## 📐 Example YAML

```yaml
shackles:
  - id: sh1
    model: GP55
    pin_connection: spreader.left_upper
    rotation_about_pin: 90 deg

elements:
  - id: sling_1
    ap1: spreader.left_upper
    ap2: hook.hook
    Lultimate: 10.0 m
    k: 8000 kN/m
```

---

## 🔩 Core Concepts

### Attachment Points

Connections are defined via attachment points:

- Local position
- Optional axis (for pins)
- Type (hole, etc.)

---

## ⚙️ Solver

The solver builds the system and runs a dynamic simulation to equilibrium using Exudyn.

---

## 📊 Output

- Positions and rotations
- Sling forces
- Residuals

---

## ✅ Status

- Stable for complex lifting cases
- Supports multiple shackles and sling systems

---

🤝 Contributing
This project is under active development. Contributions and feedback are welcome.

---

## 🔗 Dependencies

This project uses:

- [Exudyn](https://github.com/jgerstmayr/EXUDYN) — multibody dynamics solver backend

For installation and usage, refer to the Exudyn documentation:
https://exudyn.readthedocs.io/
