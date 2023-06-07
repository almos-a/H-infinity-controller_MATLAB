# H-infinity-controller_MATLAB
The H-infinity control problem solution seeks to find a controller, K wgich minimises the H-Inf norm of a closed loop transfer function.
The H-infinity suboptimal control problem solution sloution simply seeks to find the controller, K such that the H-inf norm of the closed-loop tf satisfies a condition of being less than or equal to a goptimum value.
The optimal problen therefore aims at finding the minimum value of goptimum.

The H-Inf_ctrlr_Aircraft_model.m code example follows from a text example for the vertical-plane dynamics of an aircaft. It has 3 inputs, 3 outputs and 5 states.
The code also shows how sensitivity functions can be shaped with weighting filters.
The designed controller is robust to additive perturbations, and presents fast tracking of step changes for all reference inputs, with little to no overshoot.


For contributions, corrections, comments, please feel free to contact
