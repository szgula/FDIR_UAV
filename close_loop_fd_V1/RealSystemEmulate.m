function [y,real_system] = RealSystemEmulate(real_system, u, T, dt)

t = [0; dt];
last_x = real_system.last_x;
a_fault = real_system.actuator_fault;
s_fault = real_system.sensor_fault;
model = real_system.model;

% Add actuator faults (rudder)
[u_with_artefacts, actuator_fault] = SimulateActuatorFault(u, T, a_fault);

% Emulate the real system
model_input = [u_with_artefacts; u_with_artefacts];
[y, ~, x_next] = lsim(model, model_input, t, last_x, 'zoh');
x_next = x_next(2, :);
y = y(2, :);

y_no_faults = y;
% Add noise to heading sensor
if real_system.f_add_noise
    y(3) = y(3) + (rand() - 0.5) * 2 * deg2rad(8);
end

% Add sensor faults (heading sensor)
[y, output_fault] = SimulateInputFault(y, T, s_fault);

% Update last system state
real_system.last_x = x_next;


% Logging
real_system.input_history = [real_system.input_history; u];
real_system.output_history = [real_system.output_history; y];
real_system.input_with_faults_history = [real_system.input_with_faults_history;
                                         u_with_artefacts];
real_system.output_without_faults_history = [real_system.output_without_faults_history;
                                             y_no_faults];
real_system.internal_state_history = [real_system.internal_state_history;
                                      x_next];
real_system.actuator_faults_history = [real_system.actuator_faults_history;
                                       actuator_fault];
real_system.sensor_faults_history = [real_system.sensor_faults_history;
                                    output_fault];                                  

end



