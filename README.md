![Unit Tests and Linter](https://github.com/antontmoore/boilers_control/actions/workflows/python-app.yml/badge.svg)

# Microgrid Controller for 5 houses

This repository is a solution for the AmpX test task.

## The task
![task](/images/task.png)

## 1. Simulation
A separate model has been created for each house, which takes into account:
- increase in temperature due to heating (when the boiler is on)
- heat loss caused by water consumption
- the fact that not all consumed energy is converted into heat, i.e. efficiency factor
- shutdown of the boiler when the set point is reached with a margin

## 2. Ways of solution
Several controllers have been created, each of which satisfies the given constraints.

#### 🔥 Always-on controller
All boilers are always on. Used as a baseline to estimate savings in money and electricity.

#### 💰 Naiive  controller

Uses a greedy strategy, turns on all boilers alternately, while satisfying all existing restrictions. Does not use electricity price information. 

#### 🚀 Dynamic programming
The problem of dynamic programming is solved. Each time the controller is called, optimal control is generated for a predetermined horizon. In this case, the entire horizon is divided into 15-minute periods, which are then interpolated.

#### 🤖 Model predictive control
Similar to dynamic programming, it has a horizon for which it tries to solve the optimal control problem. By varying two variables (start and end times of heating) minimizes the optimized metric.

#### Metrics

It is necessary to minimize the electricity bill and at the same time maintain a comfortable temperature. In this regard, the following metric was chosen:
$$ 
J = c_{1} \cdot price \cdot E_{consumed} + c_{2} \cdot Q_{water} \cdot (T_{setpoint} - T) + c_{3} \cdot (T_{setpoint} - T)
$$

where 
$ E_{consumed} $ - energy consumed by all boilers with current $ price $, $Q_{water}$ - water consumption, $T$ - current temperature, $T_{setpoint}$ - setpoint value of temperature, $c_{1}, c_{2}, c_{3}$ - coefficients.

## 3. Test envorinment
A special environment for checking and testing controllers has been created.

![test environment](/images/test%20environment.jpg)

To start the testing environment, you simply need to run `python dash_app.py`
