<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-09-07 | Updated: 2026-09-07 -->
# web/ (web_node dashboard UI)

## Purpose
Single-page browser UI served by `move_control/web_node.py` (installed to
`share/move_control/web/`). Polls `/state.json` every 0.5 s and `/map.png`
only when its gen counter changes; posts to `/cmd`, `/wander`, `/estop`,
`/teleop`.

## Key Files
| File | Description |
|---|---|
| `dashboard.html` | map canvas (PNG + trail/robot/goal/route/options overlays), control state panel (mode/estop/wander/goal/eta/ok/health/vel), goal·wander·estop buttons, hold-to-drive teleop pad |

## For AI Agents

### Working In This Directory
- Dependency-free on purpose: no framework, no build step — one HTML file,
  one IIFE, served straight from the robot's share dir.
- Design tokens live in the `:root` CSS block and the `T` object in the
  script (they must stay in sync); dataviz reference palette, dark. Overlay
  hues are validated categorical slots (route blue / alt orange / goal
  aqua); red is reserved for status/critical; `--pin` for the user pin.
- The PNG palette in `web_node.render_png` must match the canvas tokens.
- The map view is base-fit composed with user zoom `{z, A}` (A = screen
  anchor of cell 0). Pan is clamped so the raster never leaves the canvas;
  a map-meta change re-anchors the world point at the canvas center. Overlay
  strokes stay screen-constant (`* dpr`, never `* z`).
- All controls bind via addEventListener (`data-post`, `data-hold`,
  `data-view`) — no inline onclick; element ids are the poll()/draw()
  contract, add ids and JS together.
- Config constants live in the CFG block (poll 333 ms, teleop 100 ms,
  zoom 1-8x, stale 1.5 s). Teleop clamps must match web_node's /teleop.
