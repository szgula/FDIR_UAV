clear; close all;
p = 0; % roll rate
r = 0; % yaw rate
psi = 0; % yaw angle
phi = 0; % roll angle
beta = 0; % sidesplip angle
delta_a = 0; % aileron actuator deflection <-60deg, 60deg>
delta_r = 0; % rudder actuator deflection


x = [p; r; psi; phi; beta];
u = [delta_a; delta_r];

A = [-11.4540, 2.7185, 0, 0, -19.4399;
    0.5068, -2.9875, 0, 0, 23.3434;
    0, 1, 0, 0, 0;
    1, 0.0926, 0, 0, 0;
    0.0922, -0.9957, 0, 0.3256, -0.4680];

B = [78.4002, -2.7282;
    -3.4690, 13.9685;
    0, 0;
    0, 0;
    0, 0];

C = [1, 0, 0, 0, 0;
    0, 1, 0, 0, 0;
    0, 0, 1, 0, 0;
    0, 0, 0, 1, 0;
    0, 0, 0, 0, 0];

D = [0, 0;
    0, 0;
    0, 0;
    0, 0;
    0, 0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% continous model
model = ss(A, B, C, D, 'StateName',{'p' 'r', 'psi', 'phi', 'beta'},'InputName',{'delta_a', 'delta_r'});
x0 = [0;0;0;0;0];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% discerte model
dt = 0.01;                  % simulation step [s]
T = 200;                     % simulation time [s]

model = c2d(model, dt);

[input_r,t] = gensig("sine",1,T,dt);

x_last = x0;
y_iter = x0;
x_state = [x0'];
y = [x0'];
last_input = [0,0];
control_signel = [];
faults = [];

target = deg2rad(20);
controller_data = init_controller(dt);
FD_internal = InitFaultDetector();

for i=1:length(t)-1
    if y_iter(3) > deg2rad(20) && target == deg2rad(20)
        target = -deg2rad(20);
    elseif y_iter(3) < -deg2rad(20) && target == -deg2rad(20)
        target = deg2rad(20);
    end
    
    [controller_input, controller_data] = zig_zag_controller_with_roll_stabilization_smooth(y_iter, target, controller_data);
    [controller_input, actuator_fault] = SimulateActuatorFault(controller_input, i*dt, "driftwise");
    model_input = [last_input; controller_input];
    [y_iter, ~, x_next] = lsim(model, model_input, t(1:2), x_last, 'zoh');  % t(i:i+1) gives warning
    x_next = x_next(2, :);
    y_iter = y_iter(2, :);
    %y_iter(3) = y_iter(3) + (rand() - 0.5) * 2 * deg2rad(8);
    
    
    [y_iter, input_fault] = SimulateInputFault(y_iter, i*dt, "driftwise0");
    FD_internal = FaultDetection(y_iter, controller_input, FD_internal);
    FD_internal = FaultEstimation(FD_internal);
    %%%%%% LOGING %%%%%%%
    x_state = [x_state; x_next];
    y = [y; y_iter];
    control_signel = [control_signel; controller_input];
    faults = [faults; [actuator_fault, input_fault]];
    
    %%%%%%% UPDATE "LAST" PAREMETER %%%%%%%
    x_last = x_next;
    last_input = controller_input;
end



%initial(model,x0)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

PlotSystemBehaviour(x_state, y, control_signel, t);
figure
plot(t(1:length(FD_internal.history)), FD_internal.history);
hold on
plot(t(1:length(FD_internal.control_history)), FD_internal.history);
title("Run sum: Detection of change in mean ");
legend("p", "r", "psi", "phi", "beta", "aileron control", "rudder control");
xlabel("time [s]");
ylabel("Fault KPI");
figure
hold on
plot(t(1:length(FD_internal.history)), FD_internal.rs_f_status_history, '--', 'LineWidth', 1);
%plot(t(1:length(FD_internal.history)), FD_internal.control_rs_f_status_history, '--', 'LineWidth', 1);
plot(t(1:length(faults)), faults, 'LineWidth', 2);
title("Faults");
legend("p - fault flague", "r - fault flague", "psi - fault flague", "phi - fault flague", "beta - fault flague", "actuator fault", "sensor fault");
xlabel("time [s]");
ylabel("Fault value [rad] & Fault flags [-]");


figure
hold on
plot(t(1:length(FD_internal.fault_detection_slope)), FD_internal.fault_detection_slope, '-', 'LineWidth', 1);
title("Faults estimation - slope");
grid on
legend("p - fault flague", "r - fault flague", "psi - fault flague", "phi - fault flague", "beta - fault flague");
xlabel("time [s]");
ylabel("Fault slope");
% title("Faults estimation - slope (sensor stepwise, a=1)");
