mu_set = [0.8, 0.50];
n = length(mu_set);
close all;

for i=(1:n)
    subplot(round(n/2), 2, i);
    mu = mu_set(i);
    sim('closed_loop');
    % Prepare output for simulation_saver
    out = ans;
    simulation_saver;
    plot(states(:,1), states(:,2));
    title('\mu=', mu_set(i));
    hold on;
    plot(reference(:,1), reference(:,2), '--');
    axis equal;
end