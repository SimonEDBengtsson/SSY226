clc;
ts = 0.05; %s
m = 1; %kg
Iz = 1; %kg*m^2
x0 = [0 0 0 0 0 0];

A_c = [0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1;
     0 0 0 0 0 0;
     0 0 0 0 0 0;
     0 0 0 0 0 0];
 
B_c = [zeros(3,3); diag([1/m 1/m 1/Iz])];

C_c = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 0 0 0];


D_c = 0;

sys = ss(A_c, B_c, C_c, D_c);

sys_d = c2d(sys, ts);

mpc_controller = mpc(sys_d);

mpc_controller.Weights.OutputVariables = [10 10 10];
mpc_controller.Weights.ManipulatedVariables = [1 1 1];
mpc_controller.Weights.ManipulatedVariablesRate = [1 1 1];
mpc_controller.ControlHorizon = 5;
mpc_controller.PredictionHorizon = 20;

mpc_controller

%% Animation

y = out.yout.get(1);
y = y.Values.Data;
t = out.tout;
size = [1 1];
clear frames;

plot(1); clf; grid on; hold on; axis equal;
rect = rectangle('position', [-size/2 size], 'curvature', [1 1]);
arrow = quiver(0, 0, 1*sin(0), 1*cos(0), 'linewidth', 3);
axis([-3 3 -2 12]);

for x = 1:length(t)
    frames(x) = getframe;
    rect.Position = [y(x,1)-size(1)/2, y(x,2)-size(2)/2, size];
    arrow.XData = y(x,1);
    arrow.YData = y(x,2);
    arrow.UData = sin(y(x,3));
    arrow.VData = cos(y(x,3));
end
%%

movie(frames, 1, length(t)/10)