function [real_system] = InitRealSystem(model, x0, actuator_fault, sensor_fault, f_add_noise)

real_system.model = model;
real_system.sensor_fault = sensor_fault;
real_system.actuator_fault = actuator_fault;
real_system.last_x = x0;
real_system.f_add_noise = f_add_noise;

real_system.input_history = [];
real_system.output_history = [];
real_system.input_with_faults_history = [];
real_system.output_without_faults_history = [];
real_system.internal_state_history = [x0'];
real_system.actuator_faults_history = [];
real_system.sensor_faults_history = [];
end

