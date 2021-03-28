function [fault_detector] = FaultDetectorAndEstimator(fault_detector, x_hat, y, controller_output, t, dt)

B = fault_detector.model.B;
A = fault_detector.model.A;
x_hat_last = fault_detector.x_hat_last;

%% Invese Simulation
% Dummy inverse simulation
u_pred = B \ ((x_hat - x_hat_last)'/dt - A*x_hat_last');
fault_detector.actuator_fault = u_pred' - controller_output;   

% Inverse simulation logging
fault_detector.predicted_system_input = [fault_detector.predicted_system_input;
                                         u_pred'];                                  
fault_detector.estimated_actuator_fault = [fault_detector.estimated_actuator_fault;
                                           fault_detector.actuator_fault];


                                       
                                       
%% Output Error Residual 
y_pred = x_hat(1:4);
r_prim_val = y - y_pred;
fault_detector.r_prim_hist = [fault_detector.r_prim_hist; r_prim_val];


%% Calculate sensor fault

% filter r_prim
filter_order = fault_detector.filter_order;
if length(fault_detector.r_prim_hist) > filter_order
    filt_all = filter(fault_detector.b_filter, 1, fault_detector.r_prim_hist(end-filter_order:end, :));
    fault_detector.r_prim_filtered_hist = [fault_detector.r_prim_filtered_hist; filt_all(end, :)];
else
    fault_detector.r_prim_filtered_hist = [fault_detector.r_prim_filtered_hist; mean(fault_detector.r_prim_hist, 1)];
end
    

sensor_fault_val = fault_detector.r_prim_filtered_hist(end, 3);
fault_detector.estimated_sensor_fault = [fault_detector.estimated_sensor_fault; 
                                         sensor_fault_val];

% Detect sensor fault - flag                                   
if fault_detector.f_sensor_fault == false && abs(sensor_fault_val) > deg2rad(10)
    fault_detector.sensor_fault_time = t;
    fault_detector.f_sensor_fault = true;
end

% Estimate sensor fault   
if fault_detector.f_sensor_fault && (t == (fault_detector.sensor_fault_time + fault_detector.sensor_sample_delay))
    fault_detector.first_sampled_sensor_fault = sensor_fault_val;
    
elseif fault_detector.f_sensor_fault && (t == (fault_detector.sensor_fault_time + fault_detector.sensor_compensation_time + fault_detector.sensor_sample_delay))
    fault_detector.sensor_compensation_slope = (sensor_fault_val - fault_detector.first_sampled_sensor_fault) / fault_detector.sensor_compensation_time;
    fault_detector.sensor_fault_estimated = true;
    
    % assuming step fault
    if abs(fault_detector.sensor_compensation_slope) < deg2rad(0.4)
        fault_detector.sensor_compensation_slope = 0;
        fault_detector.sensor_compensation_constant = fault_detector.first_sampled_sensor_fault;
    % assuming drift fault
    else
        fault_detector.sensor_fault_time = fault_detector.sensor_fault_time - fault_detector.first_sampled_sensor_fault / fault_detector.sensor_compensation_slope;
        fault_detector.sensor_compensation_constant = 0;
    end
end

fault_detector.x_hat_last = x_hat;

end

