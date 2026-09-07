<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-09-07 | Updated: 2026-09-07 -->
# web/ (web_node dashboard UI)

## Purpose
Single-page browser UI served by `move_control/web_node.py` (installed to
`share/move_control/web/`). Polls `/state.json` every 333 ms and `/map.png`
only when its gen counter changes; posts to `/cmd`, `/wander`, `/estop`,
`/goal`, `/teleop`, `/map/reset`, `/map/resume`.

## Key Files
| File | Description |
|---|---|
| `dashboard.html` | the whole console: three regions (감지 / 관측 / 조작), map canvas with overlays, lidar dial, clearance gauges, safety flags, camera, and every control |

## For AI Agents

### Working In This Directory
- Dependency-free on purpose: no framework, no build step, no webfonts — one
  HTML file, one IIFE, served straight from the robot's share dir with no
  internet. Personality comes from scale/weight/spacing, not typeface.
- **Three regions, not a card stack.** `감지` (sense) reports, `관측`
  (observe) is the map hero, `조작` (act) is the only place that moves the
  robot. Regions are told apart by ground colour and the `.rhead` rule —
  `.act` is the raised surface and the only region with bordered blocks.
  Keep anything that commands the robot in `조작`.
- **Distances are shown in centimetres**, against the robot's own body. The
  maze is desk-scale: 12 cm stop, 7.6 cm chassis, 2 cm US blind zone. Bare
  metres do not read at a glance. Gauges stay neutral grey and only take a
  status colour when a threshold is crossed, so colour always means
  something.
- Thresholds come from `state.limits`, which `web_node.read_limits` reads
  from `config/robot.yaml` (`LIMIT_PARAMS` / `LIMIT_KEYS`) — the same
  numbers safety runs on. `CFG.lim` is only the no-safety-node fallback;
  never hardcode a second copy of a stop distance.
- Flags have polarity: `bad:true` means true is the alarm (cliff, tilt);
  without it, true is the good state (rear clear, can reverse). Off-states
  stay rendered so "no cliff" is visible rather than merely absent.
- The nose bearing is `scan.nose_yaw`, resolved by web_node from the
  base←scan TF — the same one safety uses. If it is missing the page refuses
  to guess a mount angle (`noseYaw` returns null, `scan_reason` explains).
- Design tokens live in the `:root` CSS block and the `T` object in the
  script (they must stay in sync); dataviz reference palette, dark. Overlay
  hues are validated categorical slots (route blue / alt orange / goal
  aqua); red is reserved for status/critical; `--pin` for the user pin.
- The PNG palette in `web_node.render_png` must match the canvas tokens.
- The map view is base-fit composed with user zoom `{z, A}` (A = screen
  anchor of cell 0). Pan is clamped so the raster never leaves the canvas;
  a map-meta change re-anchors the world point at the canvas center. Overlay
  strokes stay screen-constant (`* dpr`, never `* z`). The robot is drawn as
  a true-scale body circle (`limits.radius`), so gaps are judged in body
  widths.
- All controls bind via addEventListener (`data-post`, `data-hold`,
  `data-view`) — no inline onclick; element ids are the poll()/draw()
  contract, add ids and JS together.
- Config constants live in the CFG block (poll 333 ms, teleop 100 ms,
  zoom 1-8x, stale 1.5 s, camera stale 3 s). Teleop clamps must match
  web_node's `/teleop`, and `data-post` values must be in its `POST_VERBS`.
- Canvas draw functions must early-return on a zero-size canvas: the node
  tests in `tools/test_dashboard_*.cjs` run the IIFE with stub elements.
