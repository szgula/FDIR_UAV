function [FD_internal] = FaultDetection(y, u, FD_internal)

% SENSOR INPUT
dy = y - FD_internal.reference_mean;
sigma_range = dy ./ FD_internal.reference_std;

sigma_range = sign(sigma_range) .* min(3, floor(abs(sigma_range)));

% calculate running sum (rs) of sigma distances
FD_internal.run_sum = FD_internal.run_sum + sigma_range;
FD_internal.history = [FD_internal.history; FD_internal.run_sum];

FD_internal.rs_f_status = FD_internal.rs_f_status | abs(FD_internal.run_sum) > FD_internal.run_sum_thresholds;
FD_internal.rs_f_status_history = [FD_internal.rs_f_status_history; FD_internal.rs_f_status];


% CONTROLLER OUTPUT
du = u - FD_internal.control_mean;
sigma_range = du ./ FD_internal.control_std;

sigma_range = sign(sigma_range) .* min(3, floor(abs(sigma_range)));

% calculate running sum (rs) of sigma distances
FD_internal.control_run_sum = FD_internal.control_run_sum + sigma_range;
FD_internal.control_history = [FD_internal.control_history; FD_internal.run_sum];

FD_internal.control_rs_f_status = FD_internal.control_rs_f_status | abs(FD_internal.control_run_sum) > FD_internal.control_run_sum_thresholds;
FD_internal.control_rs_f_status_history = [FD_internal.control_rs_f_status_history; FD_internal.control_rs_f_status];
end

