<!-- PROJECT TITLE -->
<h1 align="center">🧠 ProbabilisticControlSim</h1>
<p align="center">
  <em>Probabilistic Cost Analysis in Stochastic Control Systems</em>  
  <br>
  <strong>Submitted to IEEE Control Systems Letters (L-CSS), 2025</strong>
</p>

---

<p align="center">
  <a href="https://www.python.org/"><img src="https://img.shields.io/badge/python-3.10%2B-blue.svg?logo=python&logoColor=white" alt="Python Version"></a>
  <a href="https://opensource.org/licenses/MIT"><img src="https://img.shields.io/badge/license-MIT-green.svg" alt="License: MIT"></a>
  <a href="https://ieeexplore.ieee.org/"><img src="https://img.shields.io/badge/IEEE-L--CSS-lightgrey.svg?logo=ieee" alt="IEEE L-CSS"></a>
  <a href="https://github.com/AdrianSzklarski/ProbabilisticControlSim/actions"><img src="https://img.shields.io/badge/build-passing-brightgreen.svg?logo=github" alt="Build Status"></a>
  <a href="#"><img src="https://img.shields.io/badge/status-active-success.svg" alt="Status"></a>
</p>

---

## 📘 Overview

This repository provides a **Python implementation and simulation framework** for analyzing **probabilistic control cost** in **stochastic dynamic systems**.

It integrates:
- 🎯 **Linear Quadratic Regulator (LQR)** control  
- 🔍 **Kalman Filter**–based state estimation  
- 🎲 **Monte Carlo Simulation**

to evaluate:
- Expected cost: **E[J]**  
- Variance: **Var[J]**  
under random process and measurement disturbances.

> The project connects *probabilistic runtime analysis* with *stochastic optimal control* — quantifying how randomness influences stability, cost, and performance.

---

## 🧩 System Model

The stochastic system follows:

\[
\dot{x} = A x + B u + \eta(t)
\]
\[
y = C x + v(t)
\]

where:

| Symbol | Description |
|---------|-------------|
| \( x = [position, velocity]^T \) | System state |
| \( u = -K \hat{x} \) | LQR control law |
| \( \eta(t) \sim \mathcal{N}(0, Q_w) \) | Process noise |
| \( v(t) \sim \mathcal{N}(0, R_v) \) | Measurement noise |
| \( \hat{x} \) | Kalman filter estimate |

The **cost function**:
\[
J = \int (x^T Q x + u^T R u) \, dt
\]

is approximated numerically over the simulation horizon.

---

## ⚙️ Simulation Parameters

| **Parameter** | **Value** |
|----------------|-----------|
| Simulation time (T) | 5.0 s |
| Time step (dt) | 0.01 s |
| Monte Carlo trials | 300 |
| Process noise covariance (Qw) | 1e-4 × I |
| Measurement noise covariance (Rv) | 1e-2 × I |
| Cost matrices (Q, R) | Q = diag(10, 1), R = [0.1] |

---

## 🧮 Core Python Script

**Main file:** `montecarlo_lqr_kalman.py`

### Example usage
```python
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

results = monte_carlo_simulation_vectorized(
    A, B, C, Qw, Rv, Qcost, Rcost,
    x0, T=5.0, dt=0.01, trials=300
)

print("Mean cost:", results["mean"])
print("Variance:", results["var"])
print("95% CI:", results["ci95"])

