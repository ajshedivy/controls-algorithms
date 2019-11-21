import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
from driver import Driver
import numpy as np
import math
from random import randint
from path import PathTracker, Path, RandomPathGenerator

class Model(object):
    def __init__(self, render):
        self.render = render

        self.observation_space = np.empty([9, 1])
        self.action_space = np.empty([3,])
        self.info = {}
        self.timestep = 0.01
        # ---------------------------------------------------------------------
        #
        #  Create the simulation system and add items
        #

        self.timeend = 30

        # Create the vehicle system
        chrono.SetChronoDataPath("/home/aaron/chrono/data/")
        veh.SetDataPath("/home/aaron/chrono/data/vehicle/")

        # JSON file for vehicle model
        self.vehicle_file = veh.GetDataPath() + "hmmwv/vehicle/HMMWV_Vehicle.json"

        # JSON files for terrain
        self.rigidterrain_file = veh.GetDataPath() + "terrain/RigidPlane.json"

        # JSON file for powertrain (simple)
        self.simplepowertrain_file = veh.GetDataPath() + "generic/powertrain/SimplePowertrain.json"

        # JSON files tire models (rigid)
        self.rigidtire_file = veh.GetDataPath() + "hmmwv/tire/HMMWV_RigidTire.json"

        # Initial vehicle position
        self.initLoc = chrono.ChVectorD(-125, -130, 0.5)

        # Initial vehicle orientation
        self.initRot = chrono.ChQuaternionD(1, 0, 0, 0)

        # Rigid terrain dimensions
        self.terrainHeight = 0
        self.terrainLength = 300.0  # size in X direction
        self.terrainWidth = 300.0  # size in Y direction

        # Point on chassis tracked by the camera (Irrlicht only)
        self.trackPoint = chrono.ChVectorD(0.0, 0.0, 1.75)

        self.dist = 5.0

        self.generator = RandomPathGenerator(width=100, height=100, maxDisplacement=2, steps=1)

        self.tracknum = 0


    def reset(self):
        print("reset")

        self.generator.generatePath(difficulty=50, seed=randint(1,1000))
        self.path = Path(self.generator)
        self.path_tracker = PathTracker(self.path)
        self.initLoc = self.path_tracker.GetInitLoc()
        self.initRot = self.path_tracker.GetInitRot()

        self.vehicle = veh.WheeledVehicle(self.vehicle_file, chrono.ChMaterialSurface.NSC)
        self.vehicle.Initialize(chrono.ChCoordsysD(self.initLoc, self.initRot))
        self.vehicle.SetStepsize(self.timestep)
        self.vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
        self.vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
        self.vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
        self.vehicle.SetWheelVisualizationType(veh.VisualizationType_NONE)

        # Create and initialize the powertrain system
        self.powertrain = veh.SimplePowertrain(self.simplepowertrain_file)
        self.vehicle.InitializePowertrain(self.powertrain)

        # Create the ground
        self.terrain = veh.RigidTerrain(self.vehicle.GetSystem(), self.rigidterrain_file)

        for axle in self.vehicle.GetAxles() :
            tireL = veh.RigidTire(self.rigidtire_file)
            self.vehicle.InitializeTire(tireL, axle.m_wheels[0], veh.VisualizationType_MESH)
            tireR = veh.RigidTire(self.rigidtire_file)
            self.vehicle.InitializeTire(tireR, axle.m_wheels[1], veh.VisualizationType_MESH)

        # -------------
        # Create driver
        # -------------
        self.driver = Driver(self.vehicle)
        # Time interval between two render frames
        render_step_size = 1.0 / 60  # FPS = 60
        # Set the time response for steering and throttle inputs.
        # NOTE: this is not exact, since we do not render quite at the specified FPS.
        steering_time = 1.0
        # time to go from 0 to +1 (or from 0 to -1)
        throttle_time = 1.0
        # time to go from 0 to +1
        braking_time = 0.3
        # time to go from 0 to +1
        self.driver.SetSteeringDelta(render_step_size / steering_time)
        self.driver.SetThrottleDelta(render_step_size / throttle_time)
        self.driver.SetBrakingDelta(render_step_size / braking_time)

        vec = chrono.ChVectorD(0,0,0)
        self.path_tracker.calcClosestPoint(self.vehicle.GetVehiclePos(), vec)
        self.last_dist = vec.Length()
        self.last_throttle, self.last_braking, self.last_steering = 0,0,0


        if self.render:
            road = self.vehicle.GetSystem().NewBody()
            road.SetBodyFixed(True)
            self.vehicle.GetSystem().AddBody(road)

            num_points = self.path.getNumPoints()
            path_asset = chrono.ChLineShape()
            path_asset.SetLineGeometry(chrono.ChLineBezier(self.path_tracker.path))
            path_asset.SetColor(chrono.ChColor(0.0,0.8,0.0))
            path_asset.SetNumRenderPoints(max(2 * num_points, 400))
            road.AddAsset(path_asset)

        if self.render:
            self.app = veh.ChVehicleIrrApp(self.vehicle)
            self.app.SetHUDLocation(500, 20)
            self.app.SetSkyBox()
            self.app.AddTypicalLogo()
            self.app.AddTypicalLights(chronoirr.vector3df(-150., -150., 200.), chronoirr.vector3df(-150., 150., 200.), 100,
                                 100)
            self.app.AddTypicalLights(chronoirr.vector3df(150., -150., 200.), chronoirr.vector3df(150., 150., 200.), 100,
                                 100)
            self.app.EnableGrid(False)
            self.app.SetChaseCamera(self.trackPoint, 6.0, 0.5)

            self.app.SetTimestep(self.timestep)
            # ---------------------------------------------------------------------
            #
            #  Create an Irrlicht application to visualize the system
            #
            # ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
            # in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
            # If you need a finer control on which item really needs a visualization proxy
            # Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

            self.app.AssetBindAll()

            # ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
            # that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

            self.app.AssetUpdateAll()

        self.isdone = False
        self.steps = 0
        self.step(np.zeros(3))
        self.tracknum = self.tracknum = 0 if self.tracknum >= 3 else self.tracknum + 1
        # self.tracknum = self.tracknum + 1
        return self.get_ob()


    def step(self, ac):
        self.ac = ac.reshape((-1,))
        self.steps += 1

        if self.render:
            self.app.GetDevice().run()
            self.app.BeginScene(True, True, chronoirr.SColor(255, 140, 161, 192))
            self.app.DrawAll()
            self.app.EndScene()
        else:
            self.vehicle.GetSystem().DoStepDynamics(self.timestep)

        # Collect output data from modules (for inter-module communication)
        driver_inputs = self.driver.GetInputs()

        # Update modules (process inputs from other modules)
        time = self.vehicle.GetSystem().GetChTime()

        self.driver.Synchronize(time)
        self.vehicle.Synchronize(time, driver_inputs, self.terrain)
        self.terrain.Synchronize(time)
        if self.render:
            self.app.Synchronize("", driver_inputs)

        self.driver.SetTargetThrottle(self.ac[0,])
        self.driver.SetTargetSteering(self.ac[1,])
        self.driver.SetTargetBraking(self.ac[2,])
        # self.driver.SetTargetThrottle(1)
        # self.driver.SetTargetSteering(0)
        # self.driver.SetTargetBraking(0)

        # Advance simulation for one timestep for all modules
        self.driver.Advance(self.timestep)
        self.vehicle.Advance(self.timestep)
        self.terrain.Advance(self.timestep)
        if self.render:
            self.app.Advance(self.timestep)

        pos = self.vehicle.GetVehiclePos()
        self.rew = self.calc_rew(pos)
        self.last_throttle = self.ac[0,]
        self.last_steering = self.ac[1,]
        self.last_braking = self.ac[2,]
        self.obs = self.get_ob()

        self.is_done(pos)
        return self.obs, self.rew, self.isdone, self.info


    def get_ob(self):
        sentinel = self.vehicle.GetChassisBody().GetFrame_REF_to_abs().TransformPointLocalToParent(chrono.ChVectorD(self.dist, 0, 0))
        target = chrono.ChVectorD(0,0,0)
        self.path_tracker.calcClosestPoint(sentinel, target)
        err_vec = target - sentinel
        self.state = [
            self.vehicle.GetVehiclePos().x,
            self.vehicle.GetVehiclePos().y,
            self.vehicle.GetVehicleSpeed(),
            self.vehicle.GetVehicleRot().Q_to_Euler123().z,
            err_vec.x,
            err_vec.y,
            self.driver.GetThrottle(),
            self.driver.GetSteering(),
            self.driver.GetBraking(),
        ]
        return np.asarray(self.state)

    def calc_rew(self, pos):
        # vec = chrono.ChVectorD(0,0,0)
        # self.path_tracker.calcClosestPoint(pos, vec)
        # # self.path.GetArcLength()
        # if vec.Length() <= .1:
        #     rew = 1000
        # else:
        #     rew = 1 / vec.Length() * 100

        index = self.path.calcClosestIndex(pos)
        rew = self.path.GetArcLength(index)
        # print(rew)
        return rew

    def is_done(self, pos):
        vec = chrono.ChVectorD(0,0,0)
        self.path_tracker.calcClosestPoint(pos, vec)
        err = vec - pos
        if self.vehicle.GetSystem().GetChTime() > self.timeend:
            self.isdone = True
        elif err.Length() > 6:
            self.isdone = True

    def ScreenCapture(self, interval):
        try:
            self.app.SetVideoframeSave(True)
            self.app.SetVideoframeSaveInterval(interval)
        except:
            print("No ChIrrApp found. Cannot save video frames.")


    def __del__(self):
        if self.render:
            if hasattr(self, 'app'):
                self.app.GetDevice().closeDevice()
            print("Destructor called, Device deleted.")
        else:
            print("Destructor called, No device to delete.")
