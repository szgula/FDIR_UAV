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


x_hat_last = x0';
x_state_hat = [x0'];
y_iter_corrected = [];


controller_data = init_controller(dt);
fault_detector = InitFaultDetectorAndEstimator(model, x0);

% model, x0, actutor_fault, sensor_fault, sensor_noise
real_system = InitRealSystem(model, x0, "stepwise0", "stepwise", true); 

target = deg2rad(0);
for i=1:length(t)-1
    if 5 / dt == i
        target = deg2rad(60);
    elseif 40 / dt == i
        target = deg2rad(0);
    end
    
    [controller_output, controller_data] = conteoller_heading(x_hat_last, target, controller_data, K_p);
    [y_iter,real_system] = RealSystemEmulate(real_system, controller_output, i*dt, dt);

    
    if fault_detector.sensor_fault_estimated  % move to SensorFaultCompensator
        since_sensor_fault = i * dt - fault_detector.sensor_fault_time;
        y_iter(3) = y_iter(3) - fault_detector.sensor_compensation_constant - fault_detector.sensor_compensation_slope * since_sensor_fault;
    end
    y_iter_corrected = [y_iter_corrected; y_iter];
    
    control_kalman = [controller_output; controller_output];
    u_kalman = [control_kalman, y_iter .* ones(2, 4)];
    [x_hat, ~] = lsim(sysKf, u_kalman, t(1:2), x_hat_last);
    x_hat = x_hat(2, :);
    
    
    fault_detector = FaultDetectorAndEstimator(fault_detector, x_hat, y_iter, controller_output, i*dt, dt);
    
    %%%%%% LOGING %%%%%%%
    x_state_hat = [x_state_hat; x_hat];
    
    %%%%%%% UPDATE "LAST" PAREMETER %%%%%%%
    x_hat_last = x_hat;
end



%initial(model,x0)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

PlotSystemBehaviour(real_system.internal_state_history, x_state_hat, real_system.output_history, y_iter_corrected, real_system.input_history, t);

figure()
plot(t(1:end-1), rad2deg(fault_detector.predicted_system_input), '--')
hold on
plot(t(1:end-1), rad2deg(real_system.input_with_faults_history))
legend("predicted system input", "real input to actuator (with faults)");
title("Inverse simulation - predicted and actuall command (with faults)");
xlabel("time [s]");
ylabel("value [deg]");


figure()

plot(t(1:end-1), rad2deg(fault_detector.estimated_sensor_fault), 'LineWidth', 1);
hold on;
plot(t(1:end-1), rad2deg(real_system.actuator_faults_history), '--', 'LineWidth', 1);
legend("estimated actuator fault", "actuator fault");
xlabel("time [s]");
ylabel("value [deg]");
grid on;
title("Actuator fault");

figure()
plot(t(1:end-1), rad2deg(movmean(fault_detector.r_prim_hist, 200)));
hold on
plot(t(1:end-1), filter(fault_detector.b_filter, 1, rad2deg(fault_detector.r_prim_hist)));
title("output error residual");
grid on

figure
plot(t(1:end-1), rad2deg(fault_detector.estimated_sensor_fault),'LineWidth', 2);
hold on
plot(t(1:end-1), rad2deg(real_system.sensor_faults_history), 'LineWidth', 2);
title("Sensor fault");
legend("estimated sensor fault", "sensor fault");
xlabel("time [s]");
ylabel("value [deg]");
grid on
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