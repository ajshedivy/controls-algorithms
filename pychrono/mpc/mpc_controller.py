import pychrono as chrono


class MPCController:
    def __init__(self, vehicle, driver, path):
        self.driver = driver
        self.vehicle = vehicle
        self.path = path

        self.UpdateState()

    def UpdateState(self):
        """Updates State :: [x,y,v,heading]"""
        self.state = [
            self.vehicle.GetVehiclePos().x,
            self.vehicle.GetVehiclePos().y,
            self.vehicle.GetVehicleSpeed(),
            self.vehicle.GetVehicleRot().Q_to_Euler123().z,
        ]

    def Advance(self, step):
        # Controls algorithm here
        throttle, braking, steering = 0, 1, 0

        # To get current driver positions
        self.driver.GetThrottle()
        self.driver.GetSteering()
        self.driver.GetBraking()

        self.UpdateState()

        # Set target positions for "ecu" to attempt to get to
        self.driver.SetTargetThrottle(throttle)
        self.driver.SetTargetBraking(braking)
        self.driver.SetTargetSteering(steering)
