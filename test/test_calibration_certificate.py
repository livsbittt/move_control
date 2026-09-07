import copy
import unittest

from move_control.control.calibration_certificate import make_certificate, validate_certificate
from move_control.control.round_trip import RoundTrip


def complete_motion():
    def snapshot(x):
        return {'odom':(x,0.,0.,0.),'map_tf':(x,0.,0.),'lidar':(.65-x,),'us':(.65-x,)}
    trip=RoundTrip(0.,snapshot(0.))
    x=speed=0.
    for i in range(1,701):
        x+=speed*.05*.92
        speed=trip.update(i*.05,snapshot(x))
        if trip.done or trip.error:
            break
    assert trip.done and trip.error is None
    return trip.report()


class CalibrationCertificateTest(unittest.TestCase):
    def test_completed_simulated_roundtrip_survives_canonical_configuration_order(self):
        motion=complete_motion()
        record=make_certificate({'mount':[-.017,0.],'unit':'deg_s'},motion)
        restored=validate_certificate(record,{'unit':'deg_s','mount':[-.017,0.]})
        self.assertEqual(restored,motion)
        restored['legs'].clear()
        self.assertEqual(len(record['motion']['legs']),4)

    def test_configuration_change_and_schema_corruption_reject(self):
        record=make_certificate({'radius':.076},complete_motion())
        self.assertIsNone(validate_certificate(record,{'radius':.08}))
        for schema in (None,True,2,'1'):
            changed=copy.deepcopy(record);changed['schema']=schema
            self.assertIsNone(validate_certificate(changed,{'radius':.076}))

    def test_invalid_or_incomplete_motion_cannot_be_made_or_loaded(self):
        base=make_certificate({},complete_motion())
        mutations=[lambda m:m.update(done=False), lambda m:m['legs'].pop(),
                   lambda m:m.update(forward_scale=1.3),lambda m:m.update(reverse_scale=float('nan')),
                   lambda m:m['legs'][0].update(direction='reverse'),
                   lambda m:m['legs'][1].update(measured_m=.01),
                   lambda m:m['legs'][2].update(odom_m=.2),
                   lambda m:m['legs'][-1].update(home_error_m=.007),
                   lambda m:m.update(extra={'invalid':float('inf')})]
        for mutate in mutations:
            record=copy.deepcopy(base);mutate(record['motion'])
            self.assertIsNone(validate_certificate(record,{}))
            with self.assertRaises(ValueError):make_certificate({},record['motion'])

    def test_corrupt_records_and_nonfinite_config_reject(self):
        for record in (None,[],{},'not json'):
            self.assertIsNone(validate_certificate(record,{}))
        with self.assertRaises(ValueError):make_certificate({'bad':float('nan')},complete_motion())
