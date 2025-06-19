# Energy-Based Control of Mechanical Systems

This repository contains Python scripts developed for a university project focused on the control of mechanical systems using an **energy-based approach**, in particular the two different approaches are called **Energy-Balancing Passivity-Based Control (EB-PBC)** and **Interconnection and Damping Assignment Passivity-Based Control (IDA-PBC)**. The main application is the simulation and control of robotic manipulators, with an emphasis on passivity-based and energy-shaping control strategies.

## 🏫 Course Information

Department of Computer, Control and Management Engineering  
Sapienza University of Rome  
**Course:** Advanced Methods in Control  
**Author:** Emanuele De Bianchi  

---

## 📁 Project Overview

The scripts implement and simulate controllers for two mechanical systems:
- a two arm robotic manipulator,
- a robotic arm with a wheel attached at the tip. 

The controllers are found using the principles of energy shaping and passivity-based control (PBC). The approach leverages the system's energy (Hamiltonian) function to design stabilising and robust controllers.

Key features include:
- Numerical computation of system dynamics and energies
- Automatic differentiation using [autograd](https://github.com/HIPS/autograd)
- Simulation of closed-loop dynamics with [scipy.integrate.solve_ivp](https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html)
- Visualization of system trajectories

### 📦 Files

- `ES_PBC_manipulator.py`  
  Script for simulating a two-link manipulator with EB-PBC control. Includes system modeling, controller design, and plotting of results.
  
- `IDA_PBC_wheel.py`  
  Script for simulating a wheel-arm manipulator with IDA-PBC control. Includes system modeling, controller design, and plotting of results.

- `mylib.py`  
  Custom library for formatting `matplotlib` axes, required by both scripts.

## 🛠️ Requirements

- Python 3.8+
- numpy
- autograd
- scipy
- matplotlib

Install dependencies with:
```bash
pip install numpy autograd scipy matplotlib
```

## 🚀 Usage

1. Clone the repository.
2. Ensure all dependencies are installed.
3. Run the main script:
   ```bash
   python ES_PBC_manipulator.py
   ```
4. Adjust system and controller parameters in the script as needed.

## 📊 Results

These scripts were developed as part of a university project on advanced control of mechanical systems. The work demonstrates the effectiveness of modern control theory approaches to nonlinear mechanical systems using energy-based methods.
