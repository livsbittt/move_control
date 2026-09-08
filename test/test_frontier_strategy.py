import pytest
from unittest.mock import patch

from move_control.planning import FREE, OccupancyMap, GoalBrain
from move_control.planning.frontier import pick_goal


def clusters():
    return [dict(cell=(3,10),cells=[(3,10)],size=2),
            dict(cell=(15,10),cells=[(15,10)],size=40)]


def test_nearest_strategy_changes_target_without_changing_clearance():
    m=OccupancyMap(25,25,.1,fill=FREE)
    with patch('move_control.planning.frontier.frontier_points',return_value=clusters()):
        gain=pick_goal(m,m.grid_to_world(1,10),clear_m=.1,strategy='gain')
        near=pick_goal(m,m.grid_to_world(1,10),clear_m=.1,strategy='nearest')
    assert gain['x']==m.grid_to_world(15,10)[0]
    assert near['x']==m.grid_to_world(3,10)[0]
    assert gain['clear_m']==near['clear_m']==.1
    assert near['score']==pytest.approx(1/near['route']['length'])


def test_nearest_preserves_preferred_clearance_pass_over_shorter_retry():
    m=OccupancyMap(25,25,.1,fill=FREE)
    with patch('move_control.planning.frontier.frontier_points',return_value=clusters()), \
            patch('move_control.planning.frontier._reachable_costs',side_effect=[{(15,10):1.4},{(3,10):.2,(15,10):1.4}]), \
            patch('move_control.planning.frontier.nearest_free',side_effect=lambda inflated,cell,**kwargs:cell):
        goal=pick_goal(m,m.grid_to_world(1,10),clear_m=.2,retry_clear_m=.1,strategy='nearest')
    assert goal['x']==m.grid_to_world(15,10)[0]
    assert goal['clear_m']==.2
    assert goal['options'][0]['x']==goal['x']


def test_invalid_strategy_rejected_and_brain_default_stays_gain():
    m=OccupancyMap(25,25,.1,fill=FREE)
    with pytest.raises(ValueError):pick_goal(m,(.15,1.05),strategy='unsafe')
    with pytest.raises(ValueError):GoalBrain(frontier_strategy='unsafe')
    assert GoalBrain().frontier_strategy=='gain'
    brain=GoalBrain(frontier_strategy='nearest',clear_m=.1)
    with patch('move_control.planning.goals.pick_goal',return_value=None) as pick:
        brain.plan(m,(.15,1.05))
    assert pick.call_args.kwargs['strategy']=='nearest'
