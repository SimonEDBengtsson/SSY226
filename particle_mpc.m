clc;
ts = 0.01; %s
m = 1; %kg
Iz = 1; %kg*m^2

A = [0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1;
     0 0 0 0 0 0;
     0 0 0 0 0 0;
     0 0 0 0 0 0];
 
B = [zeros(3,3); diag([1/m 1/m 1/Iz])];

C = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 0 0 0];

D = 0;

sys = ss(A, B, C, D);

sys_d = c2d(sys, ts);

mpc_controller = mpc(sys_d);

mpc_controller.Weights.OutputVariables = [1 1 1];
mpc_controller.Weights.ManipulatedVariables = [1 1 1];
%mpc_controller.Weights.ManipulatedVariablesRate =;

mpc_controller
