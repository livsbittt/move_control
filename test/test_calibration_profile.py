import unittest
import math
from move_control.control.calibration_profile import ProfileLease, make_profile


class ProfileLeaseTest(unittest.TestCase):
    def test_explicit_recalibration_lease_retains_only_geometry_constraint(self):
        from test.test_calibration_certificate import RotationCertificateTest
        from move_control.control.rotation_envelope import RotationEnvelope
        estimator=RotationEnvelope(.08)
        for yaw in (.2,.2,-.2,-.2):
            estimator.add((0,0,yaw),(0,0,yaw),yaw,.0001)
        rotation=RotationCertificateTest.rotation()
        rotation['envelope']=estimator.report()
        lease=ProfileLease()
        lease.accept(make_profile('a',1,100.,True,(1.1,.9),'g',rotation),100.,10.,'g')
        lease.accept(make_profile('a',2,100.1,False,(1.1,.9),'g'),100.1,10.1,'g')
        self.assertIsNone(lease.rotation_envelope(10.1,.08,[]))
        trial=make_profile('a',3,100.2,False,(1.1,.9),'g',rotation_trial=True)
        self.assertTrue(lease.accept(trial,100.2,10.2,'g'))
        self.assertTrue(lease.rotation_trial_live(10.2))
        self.assertIsNotNone(lease.rotation_envelope(10.2,.08,[]))
        self.assertEqual(lease.gains(10.2),(1.,1.))
        self.assertEqual(lease.angular_gains(10.2),(1.,1.))
        self.assertIsNone(lease.rotation_envelope(12.,.08,[]))
        self.assertFalse(lease.rotation_trial_live(12.))
        lease.accept(make_profile('a',4,100.3,False,(1.,1.),'g'),100.3,10.3,'g')
        self.assertIsNone(lease.rotation_envelope(10.3,.08,[]))
        different=make_profile('a',5,100.4,False,(1.,1.),'different',rotation_trial=True)
        self.assertTrue(lease.accept(different,100.4,10.4,'different'))
        self.assertIsNone(lease.rotation_envelope(10.4,.08,[]))
        for flag,enabled in ((1,False),(True,True)):
            bad=make_profile('a',6,100.5,enabled,(1.,1.),'g',rotation_trial=flag)
            self.assertFalse(lease.accept(bad,100.5,10.5,'g'))

    def test_rotation_estimate_binds_exact_configured_geometry(self):
        from test.test_calibration_certificate import RotationCertificateTest
        from move_control.control.rotation_envelope import RotationEnvelope
        footprint=[[-.04,-.05],[-.04,.05],[.08,.05],[.08,-.05]]
        radius=math.hypot(.08,.05)
        envelope=RotationEnvelope(radius,footprint)
        for yaw in (.2,.2,-.2,-.2):
            envelope.add((0,0,yaw),(0,0,yaw),yaw,.0001)
        rotation=RotationCertificateTest.rotation()
        rotation['envelope']=envelope.report()
        lease=ProfileLease()
        self.assertTrue(lease.accept(make_profile('shape',1,100.,True,(1.,1.),'g',rotation),100.,10.,'g'))
        self.assertIsNotNone(lease.rotation_envelope(10.,radius,footprint[::-1]+footprint))
        self.assertIsNone(lease.rotation_envelope(10.,radius,[]))
        self.assertIsNone(lease.rotation_envelope(10.,radius,[[x,-y] for x,y in [[-.08,-.05],[-.08,.05],[.04,.05],[.04,-.05]]]))
        self.assertIsNone(lease.rotation_envelope(10.,radius-.0001,footprint))
        self.assertIsNone(lease.rotation_envelope(10.,radius,[[float('nan'),0]]))
        self.assertTrue(lease.rotation_estimate_required())
        self.assertIsNone(lease.rotation_envelope(12.,radius,footprint))
        self.assertTrue(lease.rotation_estimate_required())
        legacy=make_profile('shape',2,100.2,True,(1.,1.),'g')
        self.assertTrue(lease.accept(legacy,100.2,10.2,'g'))
        self.assertIsNone(lease.rotation_envelope(10.2,radius,footprint))
        self.assertTrue(lease.rotation_estimate_required())
        revoked=make_profile('shape',3,100.3,False,(1.,1.),'g')
        self.assertTrue(lease.accept(revoked,100.3,10.3,'g'))
        self.assertTrue(lease.rotation_estimate_required())
        self.assertFalse(lease.accept({},100.4,10.4,'g'))
        self.assertTrue(lease.rotation_estimate_required())
        self.assertFalse(ProfileLease().rotation_estimate_required())

    def test_untrusted_rotation_envelope_cannot_be_applied(self):
        from test.test_calibration_certificate import RotationCertificateTest
        rotation = RotationCertificateTest.rotation()
        rotation['envelope'] = {'valid': False}
        packet = make_profile('trial-a', 1, 100., True, (1., 1.), 'geometry-a', rotation)
        self.assertFalse(ProfileLease().accept(packet, 100., 10., 'geometry-a'))

    def packet(self, sequence=1, enabled=True, gains=(1.1, .9)):
        return make_profile('trial-a', sequence, 100., enabled, gains, 'geometry-a')

    def test_atomic_valid_profile_expires_and_duplicate_cannot_refresh(self):
        lease = ProfileLease()
        packet = self.packet()
        self.assertTrue(lease.accept(packet, 100., 10., 'geometry-a'))
        self.assertEqual(lease.gains(10.5), (1.1, .9))
        self.assertEqual(lease.gains(9.9), (1., 1.))
        self.assertFalse(lease.accept(packet, 100.5, 10.5, 'geometry-a'))
        self.assertEqual(lease.gains(11.6), (1., 1.))

    def test_invalid_revision_or_geometry_never_applies(self):
        lease = ProfileLease()
        packet = self.packet()
        packet['linear_gains'] = [1.25, 1.25]
        self.assertFalse(lease.accept(packet, 100., 10., 'geometry-a'))
        self.assertFalse(lease.accept(self.packet(), 100., 10., 'geometry-b'))
        self.assertEqual(lease.gains(10.), (1., 1.))

    def test_revocation_and_old_session_replay(self):
        lease = ProfileLease()
        self.assertTrue(lease.accept(self.packet(), 100., 10., 'geometry-a'))
        self.assertTrue(lease.accept(self.packet(2, False), 100., 10.1, 'geometry-a'))
        self.assertEqual(lease.gains(10.1), (1., 1.))
        new = make_profile('trial-b', 1, 100.2, False, (1., 1.), 'geometry-a')
        self.assertTrue(lease.accept(new, 100.2, 10.2, 'geometry-a'))
        self.assertFalse(lease.accept(self.packet(3), 100.2, 10.3, 'geometry-a'))

    def test_stale_latched_message_and_nonfinite_gain(self):
        lease = ProfileLease()
        self.assertFalse(lease.accept(self.packet(), 103., 10., 'geometry-a'))
        with self.assertRaises(ValueError):
            self.packet(gains=(float('nan'), 1.))
