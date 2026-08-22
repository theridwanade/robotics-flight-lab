# Robotics Flight Lab

[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

**Robotics Flight Lab** is an opensource lab environment where I conduct experiment, answer challenging questions and test theoretical knowledge on **robotics** and **aerospace engineering**. 

The goal is to experiment and learn systems behaviour, solve problem that might be encounterd in the real world and understand theoretical knowledge of robotics and aerospace right in simulations.

## Motivation
Building robotics project with physical hardware is expensive and prne to failure, especially without good understanding of the system been built - and its not like I have money to spend. This lab serves the purpose of making it easier to learn and buildl;

1. **Experimentation**: Conducting experiments and testing ideas of systems before investing in building real physical systems
2. **Research**: Finding solutions to problems that may be encountered when building physical systems, experimenting with completely new ideas and modeling its behaviour before implementation
3. **Learning**: Understanding theoretical principles through practical simulations.

## Projects

| Project | Description | Core Topics |
| :--- | :--- | :--- |
| **[1D Suspended Beam Drone](./simulations/1d_suspended_beam_drone/)** | Single-axis rotational drone stabilization under stochastic Dryden wind turbulence. | PID Control, Ziegler-Nichols Tuning, Aerodynamic Disturbance Modeling |

## Quickstart

### Prerequisites
Ensure you have [Conda](https://docs.conda.io/en/latest/) installed.

```bash
# 1. Clone the repository
git clone https://github.com/theridwanade/robotics-flight-lab
cd robotics-flight-lab

# 2. Create and activate the environment
conda env create -f environment.yml
conda activate rfl_env
```

To run a specific simulation, navigate into its folder in simulations/ and follow its local README.md.

## Contributing
Contributions, feedback, and discussions are welcome. You can get involved by:

1. Proposing a new physical model, dynamic system, or control strategy via Issues.

2. Optimizing existing solvers, tuning algorithms, or visualization routines via Pull Requests.

3. Pointing out edge-case dynamics or mathematical corrections.

To contribute code: fork the repo, create a feature branch, and submit a PR with detailed context on the changes.

## License
This project is open-source and available under the [MIT License](https://opensource.org/licenses/MIT).