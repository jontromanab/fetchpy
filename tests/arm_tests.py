#!/usr/bin/env python
PKG = 'fetchpy'
import roslib; roslib.load_manifest(PKG)
import numpy, unittest
import fetchpy


env, robot = fetchpy.initialize(sim=True)


class ArmTest(unittest.TestCase):
    def setUp(self):
        self._env, self._robot = env, robot
        self._arm = robot.arm
        self._indices = self._arm.GetArmIndices()
        self._num_dofs = len(self._indices)

    def test_SetStiffness_DoesNotThrow(self):
        self._wam.SetStiffness(0.0)
        self._wam.SetStiffness(0.5)
        self._wam.SetStiffness(1.0)

    def test_SetStiffness_InvalidStiffnessThrows(self):
        self.assertRaises(Exception, self._arm.SetStiffness, (-0.2))
        self.assertRaises(Exception, self._arm.SetStiffness, ( 1.2))

    def test_Servo_DoesNotThrow(self):
        self._arm.Servo(0.1 * numpy.ones(self._num_dofs))

    def test_Servo_IncorrectSizeThrows(self):
        velocity_small = 0.1 * numpy.ones(self._num_dofs - 1)
        velocity_large = 0.1 * numpy.ones(self._num_dofs + 1)
        self.assertRaises(Exception, self._arm.Servo, (velocity_small,))
        self.assertRaises(Exception, self._arm.Servo, (velocity_large,))

    def test_Servo_ExceedsVelocityLimitThrows(self):
        velocity_limits = self._robot.GetDOFVelocityLimits(self._indices)
        velocity_limits_small = -velocity_limits - 0.1 * numpy.ones(self._num_dofs)
        velocity_limits_large =  velocity_limits + 0.1 * numpy.ones(self._num_dofs)
        self.assertRaises(Exception, self._arm.Servo, (velocity_limits_small, 0.3))
        self.assertRaises(Exception, self._arm.Servo, (velocity_limits_large, 0.3))

    def test_Servo_InvalidAccelTimeThrows(self):
        velocity_limits = 0.1 * numpy.ones(self._num_dofs)
        self.assertRaises(Exception, self._arm.Servo, (velocity_limits, -0.1))
        self.assertRaises(Exception, self._arm.Servo, (velocity_limits,  0.0))

    def test_SetVelocityLimits_SetsLimits(self):
        velocity_limits = 0.1 * numpy.ones(self._num_dofs)
        self._arm.SetVelocityLimits(velocity_limits, 0.3)
        numpy.testing.assert_array_almost_equal(self._robot.GetDOFVelocityLimits(self._indices), velocity_limits)

    def test_MoveUntilTouch_ZeroDirectionThrows(self):
        self.assertRaises(Exception, self._arm.MoveUntilTouch, (numpy.zeros(3), 0.1))

    def test_MoveUntilTouch_ZeroDistanceThrows(self):
        self.assertRaises(Exception, self._arm.MoveUntilTouch, (numpy.array([1., 0., 0.]), 0.0))

    def test_MoveUntilTouch_NonPositiveForceThrows(self):
        self.assertRaises(Exception, self._arm.MoveUntilTouch, (numpy.array([1., 0., 0.]), 0.1, 0.))

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_arm', ArmTest)