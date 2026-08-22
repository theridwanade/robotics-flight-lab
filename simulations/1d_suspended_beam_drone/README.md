# Suspended Beam Drone with Dryden Wind
The suspended beam drone is a uniform rod suspended at the center axis of rotation - equiped with two motors for propulsion.

![Beam Drone](beam_sketch.png)

_Don't mind my sketch._

## Features
The features in this simulation project are simply the methods, simulations and questions that were answerd.
1. Basic drone model, pid calculation and control loop setup -> [model_and_pid.ipynb](notebooks/model_and_pid.ipynb)
2. Ziegler-Nichols closed loop pid tunning technique to find the optimal pid constants -> [ziegler_nichols_pid_tunner.ipynb](notebooks/ziegler_nichols_pid_tunner.ipynb)
3. Model recovery from a harsh angular disturbance using the optimal pid parameter from the `ziegler-nichols` tunning method -> [single_disturbance_with_pd_optimal_gain.ipynb](notebooks/single_disturbance_with_pd_optimal_gain.ipynb)
4. Stochastic wind generation using the dryden wind model to simulate the response of the drone model in usch turbulence -> [dryden_wind.ipynb](notebooks/dryden_wind.ipynb)
5. Comparison of all pid parameters from the [ziegler-nichols](ziegler_nichols_pid_constants.json) constants in wind and no wind condtions [pid_parameter_sweep_disturbance_sim.ipynb](notebooks/pid_parameter_sweep_disturbance_sim.ipynb)
6. Simulation configurations - realistic drone model, atmosphere and dryden wind configuration [config.json](config.json)

