function R=generate_reference(t,r_0,v_norm,turn_radius,turn_angle,l_initial,nTurns)
     l_turn=deg2rad(turn_angle)*turn_radius;
     R=zeros(length(r_0),length(t));
     R(:,1)=r_0;
     acc=0;
     for i=2:length(t)
         psi=R(3,i-1); v=v_norm(t(i-1));
         delta_t=t(i)-t(i-1);
         vx=v*cos(psi);
         vy=v*sin(psi);
         if acc<l_initial
             psi_dot=0;
         elseif (acc-l_initial)/l_turn<nTurns
             psi_dot=-v/turn_radius*(mod(fix((acc-l_initial)/l_turn),2)*2-1);
         else
             psi_dot=0;
         end
         R(1:3,i)=R(1:3,i-1)+delta_t*[vx;vy;psi_dot];
         R(4:6,i)=[vx;vy;psi_dot];
         acc=acc+vecnorm(R(1:2,i)-R(1:2,i-1));
     end
end