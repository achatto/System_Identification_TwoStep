**Aerodynamic Model Identification using the Two-Step Approach**
 
Aircraft aerodynamic model identification using flight test data using the two-step approach of system identification has
been performed.

Main script to be run : main.m
Refer to the instructions mentioned in the comments of the above file.

**File list** :

Data Files : 

*simdata2018 > da3211.m, dadoublet.m, de3211.m, dr3211.m, drdoublet.m*
Data files that were provided with the assignment. Containg flight test data for various manoeuvre.

Functions:

*aero_fm.m* : Computes aerodynamic forces and moments

*calc_f.m* : Defines state transition functions

*calc_G.m* : Computes process noise coefficient matrix

*calc_h.m* : Defines observation functions

*data_preprocessing.m* : Pre-processes flight data

*getposition.m* : Compute aircraft positions from aircraft and win velocities

*integration_rk4.m* : Fourth order Runge-Kutta integrator

*jacob.m* : Computes Jacobian of the input function

*kalman_filter.m* : Extended Kalman Filter (EKF) implementation

*LR_states.m* : Compute state variables of the linear regression model

*param_est.m* : Ordinary Least Squares parameter estimator

*param_est_alternate.m* : Parameter estimator for the alternate model structure

*param_val.m* : Validation of the computed parameter values

*param_val_alternate.m* : Validation of computed parameter values for the alternate model structure

Scripts : 

*generate_noise_vectors.m* : Generate sensor noise vectors

*main.m* : Main script

*params.m* : Import aircraft parameters


