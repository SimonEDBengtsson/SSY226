time=out.tout;
states=out.states.Data;
force_request=out.force_request.Data;
reference=generate_reference(time,R(:,1),v_norm,turn_radius,turn_angle,l_initial,nTurns)';
wheel_angles=out.wheel_angles.Data;
wheel_torques=out.wheel_torques.Data;
wheel_omega=out.wheel_omega.Data;
model_forces=out.model_forces.Data;
save([name, '.mat'],'time','states','force_request','reference','wheel_angles','wheel_torques','wheel_omega','model_forces');