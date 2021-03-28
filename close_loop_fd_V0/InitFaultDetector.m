function [FD_internal] = InitFaultDetector(model)
FD_internal.model = model;
FD_internal.dt = 0.01;
% parity equations check
FD_internal.r_prim_thresholds = [0.1, 0.15, 0.35, 10, 10];
FD_internal.history = [];
FD_internal.r_prim_sum = [0, 0, 0, 0, 0];
FD_internal.r_prim_sum_hist = [];
FD_internal.r_prim_f_status = logical([0, 0, 0, 0, 0]);
FD_internal.r_prim_f_status_history = [];

%"p fault threshold"0.1, "r fault threshold"0.15, "psi fault threshold"0.35
% x = [p; r; psi; phi; beta];

FD_internal.r_prim_sum_since_fault_estimated = [0, 0, 0, 0, 0];
FD_internal.r_prim_fault_time = [0, 0, 0, 0, 0];
FD_internal.r_prim_sum_slope = [0, 0, 0, 0, 0];
FD_internal.r_prim_sum_slope_hist = [];
FD_internal.sensor_fault = 0;
FD_internal.sensor_fault_extimation = [];

end

