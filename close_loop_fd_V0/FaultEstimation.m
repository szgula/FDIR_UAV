function [FD_internal] = FaultEstimation(FD_internal, t)

T = ones(1, 5) * t;
T_new_faults = T .* ~logical(FD_internal.r_prim_fault_time);
T_new_faults = T_new_faults .* FD_internal.r_prim_f_status;

r_prim = FD_internal.history(end, :) .* FD_internal.r_prim_f_status;


sum = r_prim + FD_internal.r_prim_sum_since_fault_estimated;
FD_internal.r_prim_sum_since_fault_estimated = sum;
FD_internal.r_prim_fault_time = FD_internal.r_prim_fault_time + T_new_faults;

slope = sum ./ (t - FD_internal.r_prim_fault_time + FD_internal.dt);
FD_internal.r_prim_sum_slope = slope;
FD_internal.r_prim_sum_slope_hist = [FD_internal.r_prim_sum_slope_hist;
                                     slope];
                                   
is_sensor_fault = (nansum(FD_internal.r_prim_f_status) == 1) * FD_internal.r_prim_f_status(3);
K_prop = 1;
sensor_fault = K_prop * slope(3) * FD_internal.dt * is_sensor_fault; %* int8((t - FD_internal.r_prim_fault_time(3)) > 0.2);
alfa = 0.1;
sensor_fault = alfa * sensor_fault + (1-alfa) * FD_internal.sensor_fault;
FD_internal.sensor_fault = sensor_fault; 
FD_internal.sensor_fault_extimation = [FD_internal.sensor_fault_extimation;
                                       sensor_fault];                                   
end
