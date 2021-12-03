front_length=1.4;
rear_length=1.6;
width=2;
height=0.35;
% Full rigid-body hub-force to particle forces matrix
r_1=[front_length;width/2;-height];
r_2=[front_length;-width/2;-height];
r_3=[-rear_length;width/2;-height];
r_4=[-rear_length;-width/2;-height];
i_hat=[1;0;0]; j_hat=[0;1;0];
r=[r_1 r_2 r_3 r_4];

B=[repmat(eye(3),[1 4])
   cross(repmat(r_1,[1 3]),eye(3),1)...
   cross(repmat(r_2,[1 3]),eye(3),1)...
   cross(repmat(r_3,[1 3]),eye(3),1)...
   cross(repmat(r_4,[1 3]),eye(3),1)];
% Front-wheel steering
% Forces in x and y, as well as yawing torque
iOutputForces=[1 2 6];
% x and y on front wheels, only x on back wheels
iInputForces=[1 2 4 5 7 8 10 11];
B_r=B(iOutputForces,iInputForces);
% Force allocation matrix
A=zeros(size(B'));
A(iInputForces,iOutputForces)=pinv(B_r);

% Parameters
radius=0.4;
mass=2000;
I=4000*eye(3);
mu=1;
turn_limit=deg2rad(30);
v_0=10;
epsilon=0.1;

% MPC
T=0.1;
N=10;
Qx=eye(6); Qu=1e-3*eye(3); Qf=1e3*eye(6);
Qu(end)=1e-6;
Ac=[zeros(3) eye(3)
    zeros(3) zeros(3)];
Bc=[zeros(3)
    diag(1./[mass mass I(end)])];
sysc=ss(Ac,Bc,[],[]);
sysd=c2d(sysc,T);
Ad=sysd.A; Bd=sysd.B;
tau=0:T:1000;
r=@(t)[1000*cos(t/100-pi/2)
       1000*(sin(t/100-pi/2)+1)
       t/100
      -10*sin(t/100-pi/2)
       10*cos(t/100-pi/2)
       1/100];
R=cell2mat(arrayfun(r,tau,'UniformOutput',false));
%% Wheel direction controller H_inf
tau=1;
G=1/tf('s');% Internal plant model
W_u=1;% Steering rate weight
W_y=0.01*filter(0.1,10,1);% Tracking error weight
P_steer=[0    W_u
         W_y -W_y*G
         1   -G];
[K_steer,N_steer,gamma]=hinfsyn(P_steer,1,1);

%% Wheel direction controller LQR
Ae=[0 0
   -1 0];
Be=[1
    0];
Q=diag([0 1]);
K_steer=lqr(Ae,Be,Q,10);

% Filter with stationary gain, high frequency gain, and crossover frequency
function H=filter(g_0,g_inf,f_c)
    w_c=2*pi*f_c;
    k=g_inf;
    z=sqrt((1-g_inf^2)/(g_0^2-1))*w_c;
    p=g_0/g_inf*z;
    H=tf(k*[1 p],[1 z]);
end