function [FD_internal] = FaultEstimation(FD_internal)

% for zig zag- we get dominant frequency at ~0.1Hz -> 10s
dt = 0.01;
% average 10s / 0.01s -> 1000 samples
slope = [0, 0, 0, 0, 0];

if sum(FD_internal.rs_f_status) > 0
    
    fault_sum = FD_internal.run_sum .* FD_internal.rs_f_status;
    past = FD_internal.fault_detection_run_sum_hist_with_fault;

    FD_internal.fault_detection_run_sum_hist_with_fault = [past; fault_sum];
    run_sim_mean = movmean(FD_internal.fault_detection_run_sum_hist_with_fault, 10000);%, 'Endpoints','discard');

    if size(run_sim_mean, 1) >= 2 && sum(FD_internal.rs_f_status) > 0
        diff = run_sim_mean(end, :) - run_sim_mean(1, :);
        slope = diff ./ (size(run_sim_mean, 1) * dt);
    end

end
FD_internal.fault_detection_slope = [FD_internal.fault_detection_slope; slope];
%FD_internal.run_sum;
%FD_internal.rs_f_status;


% CONTROLLER OUTPUT
%FD_internal.control_run_sum;
%FD_internal.control_rs_f_status;
end

