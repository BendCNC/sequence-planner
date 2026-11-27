# BendCNC sequence planner

The bend-sequence planner for the [BendCNC](https://github.com/BendCNC/BendCNC) press-brake HMI and the
[BendCNC Fusion add-in](https://github.com/BendCNC/fusion-addin).

It takes a sheet-metal part exported as `.bend`, or a STEP file, and does three things:

1. **Imports** the part (`engine/`, Python + Shapely). It rebuilds the flat pattern, the rigid
   panels and the bend zones, then makes a closed 3D mesh for every bend state.
2. **Plans** the bend sequence (`viewer/*.mjs`, Node.js). It searches the bend order, part
   orientation, tool setups and back-gauge stations. It also checks collisions between the part,
   the punch, the die, the back-gauge fingers and the machine frame through every motion phase.
3. **Visualizes** the result (`viewer/index.html`, Three.js / WebGL 2). Use it to step through
   each bend and inspect rejected candidates.

The planner only computes geometry. It never sends commands to a machine.

---

## Requirements

| | Version |
|---|---|
| Node.js | 18+ (20+ recommended) |
| Python | 3.10+ with **Shapely 2.1+** and NumPy (`pip install shapely numpy`) |
| Browser | Any with WebGL 2 (only for the viewer) |

Three.js is vendored in `viewer/vendor/` (MIT, see `THREE-LICENSE.txt`), so no network access is needed.

## Use from BendCNC (git submodule)

The HMI repository includes this planner as the `sequence-planner/` submodule:

```sh
git clone --recurse-submodules https://github.com/BendCNC/BendCNC.git
# or, in an existing clone:
git submodule update --init
```

The app looks for the planner in this order:

1. the `BENDCNC_UNFOLD_DIR` environment variable,
2. a `sequence-planner/` folder next to the executable,
3. the `sequence-planner/` submodule of the source checkout.

Set `UNFOLD_PYTHON` if the Python with Shapely isn't the default `python`.

## Command line

```sh
# 1. Import a .bend into a geometric model
python engine/import_bend_package.py --bend part.bend --output part.model.json

# 2. Plan the sequence
node tools/plan_sequence.mjs --model part.model.json --options options.json --output part.plan.json
```

`import_bend_package.py --help` lists the import options (`--sign-convention`,
`--merge-split-hinges`, `--embedded-step`, …). `engine/import_step.py` imports STEP files and needs
FreeCAD's Python.

## Viewer

```sh
python tools/serve.py            # opens http://127.0.0.1:8786/viewer/
python tools/serve.py --port 8787 --no-browser
```

The server only listens on `127.0.0.1`. Load a `.bend`, a STEP file or a prepared model JSON,
then click **Calcular secuencia de esta pieza** (the viewer UI is in Spanish). Open the viewer through the server: opening `viewer/index.html`
directly doesn't load its modules.

## Tooling catalogs

The tooling geometry the planner checks against (punches/dies and back-gauge fingers) ships as
protected catalogs `viewer/tooling.json.bct` and `viewer/fingers.json.bct`; the source drawings
are not published. The CLI tools read `<name>.json` if present, otherwise `<name>.json.bct`
(`viewer/protected-json.mjs`).

To use your own tooling, generate plain `viewer/tooling.json` / `viewer/fingers.json` from your
DXF/STEP drawings with `engine/build_tooling.py` and `engine/build_fingers.py`
(`build_fingers.py` needs FreeCAD); a plain file takes precedence over the `.bct`. The browser
viewer (`viewer/app.js`) loads only the plain JSON files.

---

## Layout

| Path | Contents |
|---|---|
| `engine/` | Python importers: `.bend`, STEP, embedded STEP, bend association, tooling builders |
| `engine/clearance_cert.c` | Optional native clearance certificate helper |
| `viewer/` | Planner core (`sequence-*.mjs`, `collision.mjs`, `solver.mjs`, …), workers and the web viewer |
| `tools/plan_sequence.mjs` | File-based planner entry point used by the HMI and the add-in |
| `tools/press-runtime*.mjs` | Optional long-lived planner process for ARM64 panels |
| `tools/serve.py` | Local viewer server |
| `tools/protect_catalog.py`, `tools/stamp_engine_revision.py` | Used by the add-in packager |
| `tests/`, `**/*.test.mjs`, `engine/test_*.py` | Unit tests |

## Tests

```sh
node --test                       # runs every *.test.mjs
python -m unittest discover -s tests
python -m unittest discover -s engine -p "test_*.py"
```

## License

Copyright (C) 2026 BendCNC — <support@bendcnc.com>

This program is free software: you can redistribute it and/or modify it under the terms of the
GNU General Public License as published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version. See [LICENSE](LICENSE).

Three.js (`viewer/vendor/`) is © three.js authors, MIT license.
