function [fault_detector] = InitFaultDetectorAndEstimator(model, x0)

fault_detector.x_hat_last = x0';
fault_detector.model = model;
fault_detector.actuator_fault = 0;
fault_detector.filter_order = 100;
fault_detector.b_filter = fir1(fault_detector.filter_order, 0.001,'low');

fault_detector.f_sensor_fault = false;
fault_detector.first_sampled_sensor_fault = 0;
fault_detector.sensor_fault_time = 0;
fault_detector.sensor_sample_delay = 1;
fault_detector.sensor_compensation_time = 10;
fault_detector.sensor_fault_estimated = false;
fault_detector.sensor_compensation_slope = 0;
fault_detector.sensor_compensation_constant = 0;

fault_detector.predicted_system_input = [];
fault_detector.estimated_actuator_fault = [];
fault_detector.estimated_sensor_fault = [];

fault_detector.r_prim_hist = [];
fault_detector.r_prim_filtered_hist = [];
end

