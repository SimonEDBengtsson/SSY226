mu_set = [0.60, 0.575, 0.55];
arch_type = ["AWD, Full individual control", "FWD, 50/50 torque split"];
arch_torque = [1 2 3 4; 1 1 0 0];
arch_steer = [1 1 1 1; 1 1 1 1];
n = length(mu_set);
m = length(arch_type);

close all;
for L=(1:m)
    torque_distribution = arch_torque(L,:);
    steerable = arch_steer(L,:);
    for i=(1:n)
        figure((L-1)*n+i); axis equal; 
        mu = mu_set(i);
        sim('closed_loop');
        % Prepare output for simulation_saver
        out = ans;
        name = [arch_type{L}(1:3), '_', num2str(mu_set(i))];
        simulation_saver;
        plot(states(:,1), states(:,2));
        hold on;
        plot(reference(:,1), reference(:,2), '--');
        title(arch_type(L), ['\mu=', num2str(mu_set(i))]);
    end
end