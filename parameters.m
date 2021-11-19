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
iInputForces=[1 2 4 5 7 10];
B_r=B(iOutputForces,iInputForces);
% Force allocation matrix
A=zeros(size(B'));
A(iInputForces,iOutputForces)=pinv(B_r);
radius=0.4;
mass=2000;
I=4000*eye(3);
mu=1;