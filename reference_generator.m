r_0=[0
     0
     0
     v_0
     0
     0];
 radius=100;
 angle=80;
 l_initial=400;
 nTurns=6;
 l_end=50;
 v_norm=@(t)10;
 
 l=l_initial+nTurns*deg2rad(angle)*radius+l_end;
 super_sample_ratio=100;
 T_gen=T/super_sample_ratio;
 acc=0;
 t_end=0;
 while acc<l
     acc=acc+v_norm(t_end)*T_gen;
     t_end=t_end+T_gen;
 end
 t=0:T_gen:t_end;
 R=zeros(length(r_0),length(t));
 R(:,1)=r_0;
 for i=2:length(t)
     x=R(1,i-1); psi=R(3,i-1); v=v_norm(t(i-1));
     vx=v*cos(psi);
     vy=v*sin(psi);
     if x<l_initial
         psi_dot=0;
     elseif (x-l_initial)/sin(deg2rad(angle))/radius<nTurns
         psi_dot=-v/radius*(mod(fix((x-l_initial)/sin(deg2rad(angle))/radius),2)*2-1);
     else
         psi_dot=0;
     end
     R(1:3,i)=R(1:3,i-1)+T_gen*[vx;vy;psi_dot];
     R(4:6,i)=[vx;vy;psi_dot];
 end
 t=t(1:super_sample_ratio:end);
 R=R(:,1:super_sample_ratio:end);