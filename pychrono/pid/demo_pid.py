import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import os
import math
import sys
import matplotlib.pyplot as plt
from pid_controller import PIDSteeringController
from pid_controller import PIDThrottleController
from driver import Driver

# =============================================================================

def main() :
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    # --------------------------
    # Create the various modules
    # --------------------------

    # Create the vehicle system
    vehicle = veh.WheeledVehicle(vehicle_file ,chrono.ChMaterialSurface.NSC)
    vehicle.Initialize(chrono.ChCoordsysD(initLoc, initRot))
    #vehicle.GetChassis().SetFixed(True)
    vehicle.SetStepsize(step_size)
    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_NONE)

    # Create the ground
    terrain = veh.RigidTerrain(vehicle.GetSystem(), rigidterrain_file)

    # Create and initialize the powertrain system
    powertrain = veh.SimplePowertrain(simplepowertrain_file)
    vehicle.InitializePowertrain(powertrain)

    # Create and initialize the tires
    for axle in vehicle.GetAxles() :
        tireL = veh.RigidTire(rigidtire_file)
        vehicle.InitializeTire(tireL, axle.m_wheels[0], veh.VisualizationType_MESH)
        tireR = veh.RigidTire(rigidtire_file)
        vehicle.InitializeTire(tireR, axle.m_wheels[1], veh.VisualizationType_MESH)

    # -------------
    # Create driver
    # -------------
    driver = Driver(vehicle)

    # Set the time response for steering and throttle inputs.
    # NOTE: this is not exact, since we do not render quite at the specified FPS.
    steering_time = 1.0;  # time to go from 0 to +1 (or from 0 to -1)
    throttle_time = 1.0;  # time to go from 0 to +1
    braking_time = 0.3;   # time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)

    # ------------------
    # Load and draw path
    # ------------------
    # path_file = 'paths/straight.txt'
    # path_file = 'paths/curve.txt'
    path_file = 'paths/NATO_double_lane_change.txt'
    # path_file = 'paths/ISO_double_lane_change.txt'
    path = chrono.ChBezierCurve.read(veh.GetDataFile(path_file))


    if irrlicht:
        road = vehicle.GetSystem().NewBody()
        road.SetBodyFixed(True)
        vehicle.GetSystem().AddBody(road)

        num_points = path.getNumPoints()
        path_asset = chrono.ChLineShape()
        path_asset.SetLineGeometry(chrono.ChLineBezier(path))
        path_asset.SetColor(chrono.ChColor(0.0,0.8,0.0))
        path_asset.SetNumRenderPoints(max(2 * num_points, 400))
        road.AddAsset(path_asset)

    # --------------------
    # Create controller(s)
    # --------------------
    steering_controller = PIDSteeringController(vehicle, driver, path)
    steering_controller.SetGains(Kp=0.4, Ki=0, Kd=0.25)
    steering_controller.SetLookAheadDistance(dist=5.0)

    throttle_controller = PIDThrottleController(vehicle, driver)
    throttle_controller.SetGains(Kp=0.4, Ki=0, Kd=0)
    throttle_controller.SetTargetSpeed(target_speed)

    # -----------------------
    # Initialize irrlicht app
    # -----------------------
    if irrlicht:
        app = veh.ChVehicleIrrApp(vehicle)

        app.SetHUDLocation(500, 20)
        app.SetSkyBox()
        app.AddTypicalLogo()
        app.AddTypicalLights(chronoirr.vector3df(-150., -150., 200.), chronoirr.vector3df(-150., 150., 200.), 100,
                             100)
        app.AddTypicalLights(chronoirr.vector3df(150., -150., 200.), chronoirr.vector3df(150., 150., 200.), 100,
                             100)
        app.EnableGrid(False)
        app.SetChaseCamera(trackPoint, 6.0, 0.5)

        app.SetTimestep(step_size);

        app.AssetBindAll()
        app.AssetUpdateAll()

    # -----------------
    # Initialize output
    # -----------------
    if output:
        try:
               os.mkdir(out_dir)
        except:
               print("Error creating directory " )

        # Generate JSON information with available output channels
        out_json = vehicle.ExportComponentList()
        print(out_json)
        vehicle.ExportComponentList(out_dir + "/component_list.json")

    # ---------------
    # Simulation loop
    # ---------------

    # Number of simulation steps between miscellaneous events
    render_steps = int(math.ceil(render_step_size / step_size))

    # Initialize simulation frame counter and simulation time
    step_number = 0
    time = 0

    if irrlicht:
        while (app.GetDevice().run()):

            # Render scene
            if step_number % render_steps == 0:
                app.BeginScene(True, True, chronoirr.SColor(255, 140, 161, 192))
                app.DrawAll()
                app.EndScene()

            # Collect output data from modules (for inter-module communication)
            driver_inputs = driver.GetInputs()

            # Update modules (process inputs from other modules)
            time = vehicle.GetSystem().GetChTime()

            driver.Synchronize(time)
            vehicle.Synchronize(time, driver_inputs, terrain)
            terrain.Synchronize(time)
            app.Synchronize("", driver_inputs)

            # Advance simulation for one timestep for all modules
            step = step_size

            # Update controllers
            steering_controller.Advance(step)
            throttle_controller.Advance(step)

            driver.Advance(step)
            vehicle.Advance(step)
            terrain.Advance(step)
            app.Advance(step)

            # Increment frame number
            step_number += 1
    else:
        while (True):
            vehicle.GetSystem().DoStepDynamics(step_size)

            # Collect output data from modules (for inter-module communication)
            driver_inputs = driver.GetInputs()

            # Update modules (process inputs from other modules)
            time = vehicle.GetSystem().GetChTime()

            driver.Synchronize(time)
            vehicle.Synchronize(time, driver_inputs, terrain)
            terrain.Synchronize(time)

            # Advance simulation for one timestep for all modules
            step = step_size

            # Update controllers
            steering_controller.Advance(step)
            throttle_controller.Advance(step)

            driver.Advance(step)
            vehicle.Advance(step)
            terrain.Advance(step)

            # Increment frame number
            step_number += 1

# =============================================================================

"""
!!!! Set this path before running the demo!
"""
chrono.SetChronoDataPath('/home/aaron/chrono/data/')
veh.SetDataPath('/home/aaron/chrono/data/vehicle/')

# JSON file for vehicle model
vehicle_file = veh.GetDataPath() +"hmmwv/vehicle/HMMWV_Vehicle.json"

# JSON files for terrain
rigidterrain_file = veh.GetDataPath() + "terrain/RigidPlane.json"

# JSON file for powertrain (simple)
simplepowertrain_file = veh.GetDataPath() + "generic/powertrain/SimplePowertrain.json"

# JSON files tire models (rigid)
rigidtire_file = veh.GetDataPath() + "hmmwv/tire/HMMWV_RigidTire.json"

# Initial vehicle position
initLoc = chrono.ChVectorD(-125, -130, 0.5)

# Initial vehicle orientation
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Rigid terrain dimensions
terrainHeight = 0
terrainLength = 300.0  # size in X direction
terrainWidth = 300.0   # size in Y direction

# Simulation step size
step_size = 3e-3

# Time interval between two render frames
render_step_size = 1.0 / 60  # FPS = 60

# Point on chassis tracked by the camera (Irrlicht only)
trackPoint = chrono.ChVectorD(0.0, 0.0, 1.75)

# Target speed [m/s]
target_speed = 12

# Simulation length (Povray only)
tend = 100.0

# Should output
output = False

# Output directories
out_dir =  "./WHEELED_JSON";

# Use irrlicht
irrlicht = True

main()
