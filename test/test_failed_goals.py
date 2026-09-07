import math
from move_control.planning import OccupancyMap,GoalBrain


def test_failed_coverage_target_is_deferred_not_marked_cleaned():
    m=OccupancyMap(30,20,.05,fill=0)
    b=GoalBrain(clear_m=.05)
    b.mode='coverage'
    pose=(.3,.3)
    first,route,_=b.plan(m,pose)
    assert first and route
    cell=m.world_to_grid(*first)
    assert cell not in b.covered
    b.avoid_goal(first)
    second,_,_=b.plan(m,pose)
    assert second is None or math.dist(first,second)>.10
    assert cell not in b.covered
    assert b._failed_goals
    b.reset()
    assert not b._failed_goals
