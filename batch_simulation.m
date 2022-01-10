%mu_set = [0.65];
mu_set = [0.65, 0.55];
%arch_type = ["FWS, 50/50 torque split", "FWI, individual front wheel control", "RWS, 50/50 torque split", "RWI, individual rear wheel control", "AWH, front split and rear individual"];
%titles = ["FWD 50/50 torque split", "FWD individual", "RWD 50/50 torque split", "RWD individual", "AWD rear individual + front 50/50 split"];
titles = ["AWD individual control and front wheel steering"];
arch_type = ["AWD, Individual control"];
%arch_torque = [1 1 0 0; 1 2 0 0; 0 0 1 1; 0 0 1 2; 1 1 2 3];
arch_torque = [1 2 3 4];
%arch_steer = [1 1 1 1; 1 1 1 1; 1 1 1 1; 1 1 1 1; 1 1 1 1;];
arch_steer = [1 1 0 0];
n = length(mu_set);
m = length(arch_type);

close all; clc;
for L=(1:m)
    torque_distribution = arch_torque(L,:);
    steerable = arch_steer(L,:);
    for i=(1:n)
        mu = mu_set(i);
        sim('closed_loop');
        %Prepare output for simulation_saver
        out = ans;
        name = [arch_type{L}(1:3), '_', num2str(mu_set(i))];
        simulation_saver;
        %name = [arch_type{L}(1:3), '_', num2str(mu_set(i))];
        %file_name = [arch_type{L}(1:3), '_', num2str(mu_set(i)), '.mat'];
        %load(file_name);
        f = figure((L-1)*n+i);
        create_plot(states, reference)
        title(titles(L), ['\mu=', num2str(mu_set(i))]);
        %title(titles(L), ['V_0=', num2str(v_0), 'm/s', ', \mu=', num2str(mu_set(i))]);
        %title(['\mu=', num2str(mu_set(i))]);
        pause(0.1) % because otherwise figures end up with wrong names sometimes
        print([name, '.eps'], '-depsc2');
    end
end

function create_plot(states, reference)
    plot(states(:,1), states(:,2), 'linewidth', 2);
        hold on;
        plot(reference(:,1), reference(:,2), 'black--', 'linewidth', 1.5);
        set(gca, 'fontsize', 12); 
        xlabel('X Pos. [m]'); ylabel('Y Pos. [m]');
        grid on; axis equal; 
end
