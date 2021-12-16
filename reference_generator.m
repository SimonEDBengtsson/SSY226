r_0=[0
     0
     0
     v_0
     0
     0];
 turn_radius=50;
 turn_angle=80;
 l_initial=100;
 nTurns=6;
 l_end=100;
 
 l=l_initial+nTurns*deg2rad(turn_angle)*turn_radius+l_end;
 super_sample_ratio=100;
 T=T_c/super_sample_ratio;
 acc=0;
 t_end=0;
 while acc<l
     acc=acc+v_norm(t_end)*T;
     t_end=t_end+T;
 end
 t=0:T:t_end;
 R=generate_reference(t,r_0,v_norm,turn_radius,turn_angle,l_initial,nTurns);
 t=t(1:super_sample_ratio:end);
 R=R(:,1:super_sample_ratio:end);