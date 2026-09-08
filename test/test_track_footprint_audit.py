import numpy as np
import pytest
from tools.gz.audit_track_footprint import separation, audit


def test_separating_axis_gap_distinguishes_touching_and_crossing_boxes():
    box=np.array([[-1.,-1.],[1.,-1.],[1.,1.],[-1.,1.]])
    assert separation(box,box+[3.,0.]) == 1.
    assert separation(box,box+[2.,0.]) == 0.
    assert separation(box,box+[1.,0.]) == -1.
    assert separation(box,box+[3.,3.]) == 1.


@pytest.mark.parametrize('samples',[
    [[0.,float('nan'),0.,0.]], [[0.,0.,0.,0.],[0.,0.,0.,0.]],
    [[1.,0.,0.,0.],[0.,0.,0.,0.]], [], [[0.,0.,0.,float('inf')]],
])
def test_invalid_observations_cannot_report_collision_free(samples):
    identity={'robot_geometry':{'footprint_xy':[[-.1,-.1],[.1,-.1],[0.,.1]]},
              'walls':[[[1.,0.,0.,0.,0.,0.],[.1,1.,1.]]]}
    with pytest.raises(ValueError):
        audit(identity,samples)
