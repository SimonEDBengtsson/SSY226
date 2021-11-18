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
mpc_controller.ControlHorizon = 3;
mpc_controller.PredictionHorizon = 10;

mpc_controller
%open('particle_model.slx');

%% Animation

y_actual = out.yout{1}.Values.Data;
y_reference = out.yout{2}.Values.Data;
t = out.tout;
size = [0.8 0.8];
plot_axis = [-1 11 -2 2];
clear frames;

plot(1); clf; grid on; hold on; axis equal; axis(plot_axis);
xlabel('x [m]'); ylabel('y [m]');
circ_actual = rectangle('position', [-size/2 size], 'curvature', [1 1], 'linewidth', 3);
arrow_actual = quiver(0, 0, 1*cos(0), 1*sin(0), 'linewidth', 3);
circ_reference = rectangle('position', [-size/2 size], 'curvature', [1 1], 'edgecolor', 'g');
arrow_reference = quiver(0, 0, 1*cos(0), 1*sin(0), 'linewidth', 2);

for x = 1:length(t)
    frames(x) = getframe;
    circ_actual.Position = [y_actual(x,1)-size(1)/2, y_actual(x,2)-size(2)/2, size];
    arrow_actual.XData = y_actual(x,1); arrow_actual.YData = y_actual(x,2);
    arrow_actual.UData = cos(y_actual(x,3)); arrow_actual.VData = sin(y_actual(x,3));
    
    circ_reference.Position = [y_reference(x,1)-size(1)/2, y_reference(x,2)-size(2)/2, size];
    arrow_reference.XData = y_reference(x,1); arrow_reference.YData = y_reference(x,2);
    arrow_reference.UData = cos(y_reference(x,3)); arrow_reference.VData = sin(y_reference(x,3));
end
%%

movie(frames, 1, length(t)/10)