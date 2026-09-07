"""Read-only frontier diagnosis from the isolated rig's captured map."""
import json
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from move_control.planning.gridmap import OccupancyMap
from move_control.planning.frontier import frontier_points, pick_goal
from move_control.planning.astar import best_route

out = Path('/tmp/pinky-calmap227')
snapshot = np.load(out/'map_grid.npz')
data = snapshot['data']
m = OccupancyMap(data.shape[1], data.shape[0], float(snapshot['resolution']), snapshot['origin'])
m.data = data.ravel().tolist()
rows = [json.loads(line) for line in (out/'stack.log').read_text().splitlines() if line.startswith('{')]
if '--events' in sys.argv:
    for row in rows[-60:]:
        decision = row.get('decision', {})
        print(row['sim_s'], row['pose'], row['wander'], decision.get('reason'),
              decision.get('safe_v'), decision.get('safe_omega'))
    raise SystemExit
pose = rows[-1]['pose']
grid = m.inflate(round(.12/m.res))
print('pose', pose, 'start free', grid.is_free(*m.world_to_grid(*pose)))
for frontier in frontier_points(m, 1)[:20]:
    route = best_route(m, pose, (frontier['x'], frontier['y']), clear_m=.12)
    viable = [p for p in frontier['cells'] if grid.is_free(*p)]
    print({key: frontier[key] for key in ('x', 'y', 'size')}, 'length', route['length'] if route else None, 'viable boundary', len(viable))
