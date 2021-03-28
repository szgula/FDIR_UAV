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
    0, 0, 0, 1, 0];

D = zeros(size(C, 1), size(B,2));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% continous model
model = ss(A, B, C, D, 'StateName',{'p' 'r', 'psi', 'phi', 'beta'},'InputName',{'delta_a', 'delta_r'});
x0 = [0;0;0;0;0];

Q = [0, 0, 0, 0, 0;
    0, 0, 0, 0, 0;
    0, 0, 0.01, 0, 0;
    0, 0, 0, 100, 0;
    0, 0, 0, 0, 0];
R = [0.1, 0; 0, 1];

K_p = lqr(model, Q, R);


%Vd = eye(5) .* [2.7282; 13.9685; 0.001; 0.001; 0.001];
Vd = eye(5) .* [100; 100; 0.001; 0.001; 0.0001];
%Vd = Vd * 1000;
% Vn = eye(4) .* [0.01, 0.01, 1.00, 0.01]';
Vn = eye(4) .* [0.0001, 0.0001, 100, 0.0001]';
Bext = [B, Vd, zeros(5,4)]; % B, disturbances, noise
Dext = [D, zeros(4,5), Vn];
model_ext = ss(A, Bext, C, Dext);
model_ref = ss(A, Bext, eye(5), zeros(5, size(Bext, 2)));

Kf = (lqr(A',C',Vd,Vn))';

sysKf = ss(A-Kf*C, [B, Kf], eye(5), 0*[B, Kf]);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dt = 0.01;                  % simulation step [s]
T = 60;                     % simulation time [s]
t = 0:dt:T;
uDist = zeros(5, size(t, 2));
uNoise = randn(4, size(t, 2)) * deg2rad(8);
u = zeros(2, size(t, 2));
uAug = [u; uDist; uNoise];

[y_ext, t] = lsim(model_ext, uAug, t);
[y_ref, t] = lsim(model_ref, uAug, t);
[x_hat, t] = lsim(sysKf, [u; y_ext'], t);


x_last = x0';
x_hat_last = x0';
y_iter_last = [0, 0, 0, 0]';
x_Kf_last = x0';
x_pred = x0';
x_state = [x0'];
x_state_hat = [x0'];
y = [y_iter_last'];
last_input = [0,0];
control_signel = [];
faults = [];
r_prim = [];
r_prim_sum = [];
r_prim_last_sum = 0;
u_pred_hist = [];
u_with_artefacts = [];
actuator_error = [];
r_prim_filtered = [];

target = deg2rad(0);
controller_data = init_controller(dt);

estimated_sensor_fault = [];
sensor_fault_time = 0;
sensor_fault = false;
sensor_fault_estimated = false;
sensor_fault_value = 0;
sensor_compensation_time = 10;
sensor_sample_delay = 1;
sensor_compensation_slope = 0;

filter_order = 100;
b_filter = fir1(filter_order,0.001,'low');


for i=1:length(t)-1
    if 5 / dt == i
        target = deg2rad(60);
    elseif 40 / dt == i
        target = deg2rad(0);
    end
    
    [controller_output, controller_data] = conteoller_heading(x_hat_last, target, controller_data, K_p);
    [control_signal_with_artefacts, actuator_fault] = SimulateActuatorFault(controller_output, i*dt, "driftwise0");
    model_input = [control_signal_with_artefacts; control_signal_with_artefacts];
    [y_iter, ~, x_next] = lsim(model, model_input, t(1:2), x_last, 'zoh');  % t(i:i+1) gives warning
    x_next = x_next(2, :);
    y_iter = y_iter(2, :);
    %y_iter(3) = y_iter(3) + (rand() - 0.5) * 2 * deg2rad(8);
    
    [y_iter, input_fault] = SimulateInputFault(y_iter, i*dt, "driftwise");
 
    
    if sensor_fault_estimated
        since_sensor_fault = i * dt - sensor_fault_time;
        
        y_iter(3) = y_iter(3) - sensor_fault_value - sensor_compensation_slope * since_sensor_fault;
    end
    
    control_kalman = [controller_output; controller_output];
    u_kalman = [control_kalman, y_iter .* ones(2, 4)];
    [x_hat, ~] = lsim(sysKf, u_kalman, t(1:2), x_hat_last);
    x_hat = x_hat(2, :);
    

    y_pred = x_hat(1:4);
    r_prim_val = y_iter - y_pred;
    r_prim = [r_prim; r_prim_val];
    if i > filter_order
        filt_all = filter(b_filter, 1, r_prim(end-filter_order:end, :));
        r_prim_filtered = [r_prim_filtered; filt_all(end, :)];
    else
        r_prim_filtered = [r_prim_filtered; mean(r_prim, 1)];
    end
    

     %% FAULT DETECTOR and ESTIMATOR
    u_pred = model.B \ ((x_hat - x_hat_last)'/dt - model.A*x_hat_last');
    u_pred_hist = [u_pred_hist; u_pred'];
    u_with_artefacts = [u_with_artefacts; control_signal_with_artefacts];
    actuator_error = [actuator_error; u_pred' - controller_output];
    
    
    estimated_sensor_fault = [estimated_sensor_fault; r_prim_filtered(end, 3)];
    if sensor_fault == false & abs(estimated_sensor_fault(end)) > deg2rad(10)
        sensor_fault_time = i * dt;
        sensor_fault = true;
    end
    if sensor_fault & (i == (sensor_fault_time + sensor_sample_delay) / dt)
        sensor_fault_value = estimated_sensor_fault(end);
    elseif sensor_fault & (i == (sensor_fault_time + sensor_compensation_time + sensor_sample_delay) / dt)
        sensor_compensation_slope = (estimated_sensor_fault(end) - sensor_fault_value) / 10;
        sensor_fault_estimated = true;
        if abs(sensor_compensation_slope) < deg2rad(0.4)
            sensor_compensation_slope = 0;
        else
            sensor_fault_time = sensor_fault_time - sensor_fault_value / sensor_compensation_slope;
            sensor_fault_value = 0;
        end
    end
    
    %%%%%% LOGING %%%%%%%
    x_state = [x_state; x_next];
    y = [y; y_iter];
    control_signel = [control_signel; controller_output];
    x_state_hat = [x_state_hat; x_hat];
    faults = [faults; [actuator_fault, input_fault]];
    
    
    %%%%%%% UPDATE "LAST" PAREMETER %%%%%%%
    x_last = x_next;
    x_hat_last = x_hat;
end



%initial(model,x0)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

PlotSystemBehaviour(x_state, x_state_hat, y, control_signel, t);

figure()
plot(t(1:end-1), rad2deg(u_pred_hist), '--')
hold on
plot(t(1:end-1), rad2deg(u_with_artefacts))
title("Inverse simulation - predicted and actuall command (with faults)");

figure()
plot(t(1:end-1), rad2deg(actuator_error(:, 2)), 'LineWidth', 1);
hold on;
plot(t(1:end-1), rad2deg(faults(:, 1)), '--',  'LineWidth', 1)
legend("estimated actuator fault", "actuator fault");
xlabel("time [s]");
ylabel("value [deg]");
grid on;
title("Actuator fault");

figure()
plot(t(1:end-1), rad2deg(movmean(r_prim, 200)));
hold on
plot(t(1:end-1), filter(b_filter, 1, rad2deg(r_prim)));
plot(t(1:end-1), rad2deg(estimated_sensor_fault));
title("output error residual");
grid on

figure
plot(rad2deg(estimated_sensor_fault));
title("estimated sensor fault");
figure(1)

% 
% figure
% plot(t(1:length(t)-1), r_prim);
% hold on
% xlabel("time [s]");
% ylabel("value");
% title("r'");
% plot([0, 60], [0.1, 0.1; 0.15, 0.15; 0.35, 0.35], "--");
% legend("p", "r", "psi", "phi", "beta", "p fault threshold", "r  fault threshold", "psi  fault threshold");
% 
% figure
% plot(t(1:length(t)-1), r_prim_sum);
% xlabel("time [s]");
% ylabel("value");
% title("cumulative r' ");
% hold on
% legend("p", "r", "psi", "phi", "beta");
% 
% figure;
% plot(t(1:length(t)-1), FD_internal.r_prim_f_status_history);
% legend("p fault", "r  fault", "psi fault", "phi fault", "beta fault");
% 
% figure
% plot(t(1:length(t)-1), FD_internal.sensor_fault_extimation);
% hold on
% plot(t(1:length(t)-1), faults(:,  2))
% legend("estimated sensor fault", "reference sensor fault");
% title("estimated fault vs reference fault");