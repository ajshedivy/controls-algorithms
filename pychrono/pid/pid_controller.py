import pychrono as chrono
import numpy as np

class PIDSteeringController():
    def __init__(self, vehicle, driver, path):
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0

        self.dist = 0
        self.target = chrono.ChVectorD(0,0,0)

        self.err = 0
        self.errd = 0
        self.erri = 0

        self.tracker = chrono.ChBezierCurveTracker(path, False)

        self.driver = driver
        self.vehicle = vehicle

    def SetGains(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def SetLookAheadDistance(self, dist):
        self.dist = dist

    def Advance(self, step):
        self.sentinel = self.vehicle.GetChassisBody().GetFrame_REF_to_abs().TransformPointLocalToParent(chrono.ChVectorD(self.dist, 0, 0))

        self.tracker.calcClosestPoint(self.sentinel, self.target)

        # The "error" vector is the projection onto the horizontal plane (z=0) of
        # vector between sentinel and target
        err_vec = self.target - self.sentinel
        err_vec.z = 0

        # Calculate the sign of the angle between the projections of the sentinel
        # vector and the target vector (with origin at vehicle location).
        sign = self.calcSign()

        # Calculate current error (magnitude)
        err = sign * err_vec.Length()

        # Estimate error derivative (backward FD approximation).
        self.errd = (err - self.err) / step;

        # Calculate current error integral (trapezoidal rule).
        self.erri += (err + self.err) * step / 2;

        # Cache new error
        self.err = err

        # Return PID output (steering value)
        steering = np.clip(self.Kp * self.err + self.Ki * self.erri + self.Kd * self.errd, -1.0, 1.0)
        self.driver.SetTargetSteering(steering)

    def calcSign(self):
        '''
        Calculate the sign of the angle between the projections of the sentinel vector
        and the target vector (with origin at vehicle location).
        '''

        sentinel_vec = self.sentinel - self.vehicle.GetVehiclePos()
        sentinel_vec.z = 0
        target_vec = self.target - self.vehicle.GetVehiclePos()
        target_vec.z = 0

        temp = (sentinel_vec % target_vec) ^ chrono.ChVectorD(0,0,1)

        return (temp > 0) - (temp < 0)

class PIDThrottleController():
    def __init__(self, vehicle, driver):
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0

        self.err = 0
        self.errd = 0
        self.erri = 0

        self.speed = 0
        self.target_speed = 0

        self.throttle_threshold = 0.2

        self.driver = driver
        self.vehicle = vehicle

    def SetGains(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def SetTargetSpeed(self, target_speed):
        self.target_speed = target_speed

    def Advance(self, step):
        self.speed = self.vehicle.GetVehicleSpeed()

        # Calculate current error
        err = self.target_speed - self.speed

        # Estimate error derivative (backward FD approximation)
        self.errd = (err - self.err) / step;

        # Calculate current error integral (trapezoidal rule).
        self.erri += (err + self.err) * step / 2;

        # Cache new error
        self.err = err

        # Return PID output (steering value)
        speed = np.clip(self.Kp * self.err + self.Ki * self.erri + self.Kd * self.errd, -1.0, 1.0)

        if speed > 0:
            # Vehicle moving too slow
            self.driver.SetTargetBraking(0)
            self.driver.SetTargetThrottle(speed)
        elif driver.GetTargetThrottle() > self.throttle_threshold:
            # Vehicle moving too fast: reduce throttle
            self.driver.SetTargetBraking(0)
            self.driver.SetTargetThrottle(driver.GetTargetThrottle() + speed)
        else:
            # Vehicle moving too fast: apply brakes
            self.driver.SetTargetBraking(-speed)
            self.driver.SetTargetThrottle(0)
