function [FD_internal] = InitFaultDetector()
FD_internal.reference_mean = [0, -0.0001, -0.001, 0, 0];
FD_internal.reference_std = [0.027, 0.192, 0.298, 0.004, 10];

FD_internal.run_sum_thresholds = [350, 350, 350, 150, 350];
FD_internal.run_sum = [0, 0, 0, 0, 0];
FD_internal.history = [];
FD_internal.rs_f_status = [0, 0, 0, 0, 0];
FD_internal.rs_f_status_history = [];


FD_internal.control_mean = [0, 0];
FD_internal.control_std = [0.051, 0.396];
FD_internal.control_run_sum_thresholds = [350, 350];
FD_internal.control_run_sum = [0, 0];
FD_internal.control_history = [];
FD_internal.control_rs_f_status = [0, 0];
FD_internal.control_rs_f_status_history = [];

FD_internal.fault_detection_run_sum_hist_with_fault = [0, 0, 0, 0, 0];
FD_internal.fault_detection_slope = [0, 0, 0, 0, 0];
end

