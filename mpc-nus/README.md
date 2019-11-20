# Model Predictive Control

MPC controller/simulator with varying simulation environments/obstacle representations. Given arrays of acceleration and heading change values, solves a soft constraint optimization problem for acceleration and heading commands to achieve desired performance. Dynamic prediction and control horizons. Adapts to new obstacles. See paper for algorithm details. 

To use, run mpc_accel.py or mpc_floorplan.py . Can click in window to place a new obstacle, model will adjust on-line. Default is to display window and let user press 'q' to move through time steps (frame-by-frame). This is a flag that can be reconfigured in the code (could also be made into an argument if desired). 

## Sample Output
#### mpc_accel.py
<img src="https://github.com/WisconsinAutonomous/controls-algorithms/blob/master/mpc-nus/pics/sample-accel.png" alt="mpc_accel.py sample output" width="80%">

#### mpc_floorplan.py
<img src="https://github.com/WisconsinAutonomous/controls-algorithms/blob/master/mpc-nus/pics/sample-floorplan.png" alt="mpc_floorplan.py sample output" width="80%">
