ProbabilisticControlSim
Probabilistic Cost Analysis in Stochastic Control Systems

This repository contains the full Python implementation and simulation results accompanying the article:
"Probabilistic Cost Analysis in Stochastic Control Systems"
submitted to IEEE Control Systems Letters (L-CSS).

📘 Project Overview

This project demonstrates a reproducible framework for analyzing the probabilistic control cost in stochastic dynamic systems.
The simulation combines:

Linear Quadratic Regulator (LQR) control,

Kalman filter–based state estimation, and

Monte Carlo simulation
to evaluate the expected cost E[J] and variance Var[J] of the control system under process and measurement noise.

The goal is to connect probabilistic runtime analysis with stochastic optimal control and to quantify how randomness affects stability and computational efficiency.

🧩 System Model

The system follows a 2D stochastic dynamic model:

x_dot = A x + B u + η(t)
y = C x + v(t)


where

x = [position, velocity]^T

u = -K * x_hat — feedback control law (LQR)

η(t) — process noise ~ N(0, Qw)

v(t) — measurement noise ~ N(0, Rv)

x_hat — estimated state from the Kalman filter

The cost function used for evaluation is:

J = ∫ (xᵀ Q x + uᵀ R u) dt


which is approximated numerically over the simulation horizon.

⚙️ Simulation Parameters
Parameter	Value
Simulation time (T)	5.0 s
Time step (dt)	0.01 s
Number of Monte Carlo trials	300
Process noise covariance (Qw)	1e-4 * I
Measurement noise covariance (Rv)	1e-2 * I
Cost matrices (Q, R)	Q = diag(10, 1), R = [0.1]
🧮 Core Python Script

Main script file: montecarlo_lqr_kalman.py

# Example of usage
from probabilistic_control import monte_carlo_simulation_vectorized

A = [[0.0, 1.0],
     [0.0, -0.5]]
B = [[0.0],
     [1.0]]
C = [[1.0, 0.0],
     [0.0, 1.0]]

Qw = [[1e-4, 0.0],
      [0.0, 1e-4]]
Rv = [[1e-2, 0.0],
      [0.0, 1e-2]]

Qcost = [[10.0, 0.0],
         [0.0, 1.0]]
Rcost = [[0.1]]

x0 = [1.0, 0.0]

results = monte_carlo_simulation_vectorized(A, B, C, Qw, Rv, Qcost, Rcost, x0, T=5.0, dt=0.01, trials=300)

print("Mean cost:", results["mean"])
print("Variance:", results["var"])
print("95% CI:", results["ci95"])


This will automatically:

compute LQR gains (via Riccati equation),

apply a discrete Kalman filter,

run 300 Monte Carlo trials,

generate convergence and distribution plots,

print summary statistics.

📊 Example Output
Mean cost: 44.7020
Variance: 12.7273
95% Confidence Interval: (44.2983, 45.1057)


Generated figures:

histogram_costs.png – distribution of J

convergence_loglog.png – convergence of mean estimator

variance_vs_trials.png – sample variance vs N

trajectories_examples.png – true vs estimated state trajectories

📁 Repository Structure
ProbabilisticControlSim/
│
├── montecarlo_lqr_kalman.py        # main simulation script
├── probabilistic_control.py         # LQR, Kalman, Monte Carlo core functions
├── results/                         # generated data and plots
├── README.md                        # this file
└── LICENSE.txt                      # license information

🚀 How to Run

Install dependencies:

pip install numpy scipy matplotlib pandas


Run the main script:

python montecarlo_lqr_kalman.py


View generated results in the results/ folder.

📈 Citation

If you use this code or framework in academic work, please cite:

[Author], "Probabilistic Cost Analysis in Stochastic Control Systems,"
IEEE Control Systems Letters (L-CSS), 2025.

📜 License

This project is released under the MIT License.
You are free to use, modify, and distribute the code with appropriate credit.

🧠 Notes

The simulation confirms that Monte Carlo mean convergence follows approximately O(1/√N).

The average cost and variance remain stable for N ≥ 100 trials.

The framework is easily extensible to higher-dimensional systems or nonlinear control problems.

Maintainer: [Your Name]
Contact: [Your Email]
Version: 1.0 — October 2025
# ProbabilisticControlSim
# ProbabilisticControlSim
