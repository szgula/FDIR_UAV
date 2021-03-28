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

Q = [0, 0, 0, 0, 0;
    0, 0, 0, 0, 0;
    0, 0, 0.01, 0, 0;
    0, 0, 0, 100, 0;
    0, 0, 0, 0, 0];
R = [0.1, 0; 0, 1];

K_p = lqr(model, Q, R);

Vn = eye(5); Vn(3,3) = 100;
Kf = (lqr(A',C',eye(5),Vn))';
sysKf = ss(A-Kf*C, [B, Kf], eye(5), 0*[B, Kf]);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% discerte model
dt = 0.01;                  % simulation step [s]
T = 60;                     % simulation time [s]


[input_r,t] = gensig("sine",1,T,dt);

x_last = x0';
y_iter_last = x0';
x_Kf_last = x0';
x_pred = x0';
x_state = [x0'];
y = [x0'];
last_input = [0,0];
control_signel = [];
faults = [];
r_prim = [];
r_prim_sum = [];
r_prim_last_sum = 0;

target = deg2rad(0);
controller_data = init_controller(dt);
FD_internal = InitFaultDetector(model);
Q_kalman = eye(2) * 0.001;
R_kalman = eye(5) * 1;
R_kalman(3,3) = 10;
[kalmf,L,~,Mx,Z] = kalman(model, Q_kalman, R_kalman);

for i=1:length(t)-1
    if 10 / dt == i
        target = deg2rad(60);
    elseif 40 / dt == i
        target = deg2rad(0);
    end
    
    y_iter_last_with_correction = y_iter_last;
    y_iter_last_with_correction(3) = y_iter_last(3) - FD_internal.sensor_fault;
    [controller_output, controller_data] = conteoller_heading(y_iter_last_with_correction, target, controller_data, K_p);
    [control_signal_with_artefacts, actuator_fault] = SimulateActuatorFault(controller_output, i*dt, "driftwise");
    model_input = [control_signal_with_artefacts; control_signal_with_artefacts];
    [y_iter, ~, x_next] = lsim(model, model_input, t(1:2), x_last, 'zoh');  % t(i:i+1) gives warning
    x_next = x_next(2, :);
    y_iter = y_iter(2, :);
    y_iter(3) = y_iter(3) + (rand() - 0.5) * 2 * deg2rad(8);
    
    [y_iter, input_fault] = SimulateInputFault(y_iter, i*dt, "stepwise0");
    %y_iter(3) = y_iter(3) - FD_internal.sensor_fault;
    
    %%%%%% PREDICTION ON MODEL %%%%%%%
    %kf_input = [controller_output, y_iter; controller_output, y_iter];
    %[y_Kf_pred, ~, x_Kf_pred] = lsim(sysKf, kf_input, t(1:2), x_Kf_last, 'zoh');
    %x_Kf_pred = x_Kf_pred(2, :);
    %y_Kf_pred = y_Kf_pred(2, :);
    
    model_input = [controller_output; controller_output];
    [y_pred, ~, x_pred] = lsim(model, model_input, t(1:2), x_pred, 'zoh');
    x_pred = x_pred(2, :);
    y_pred = y_pred(2, :);
    r_prim_val = y_iter - y_pred;
    r_prim = [r_prim; r_prim_val];
    r_prim_last_sum = r_prim_last_sum + y_iter - y_pred;
    r_prim_sum = [r_prim_sum; r_prim_last_sum];
    FD_internal = FaultDetection(r_prim_val, control_signal_with_artefacts, FD_internal);
    FD_internal = FaultEstimation(FD_internal, i*dt);
    
    
    %%%%%% LOGING %%%%%%%
    x_state = [x_state; x_next];
    y = [y; y_iter];
    control_signel = [control_signel; control_signal_with_artefacts];
    faults = [faults; [actuator_fault, input_fault]];
    
    %%%%%%% UPDATE "LAST" PAREMETER %%%%%%%
    x_last = x_next;
    y_iter_last = y_iter;
end



%initial(model,x0)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

PlotSystemBehaviour(x_state, y, control_signel, t);


figure
plot(t(1:length(t)-1), r_prim);
hold on
xlabel("time [s]");
ylabel("value");
title("r'");
plot([0, 60], [0.1, 0.1; 0.15, 0.15; 0.35, 0.35], "--");
legend("p", "r", "psi", "phi", "beta", "p fault threshold", "r  fault threshold", "psi  fault threshold");

figure
plot(t(1:length(t)-1), r_prim_sum);
xlabel("time [s]");
ylabel("value");
title("cumulative r' ");
hold on
legend("p", "r", "psi", "phi", "beta");

figure;
plot(t(1:length(t)-1), FD_internal.r_prim_f_status_history);
legend("p fault", "r  fault", "psi fault", "phi fault", "beta fault");

figure
plot(t(1:length(t)-1), FD_internal.sensor_fault_extimation);
hold on
plot(t(1:length(t)-1), faults(:,  2))
legend("estimated sensor fault", "reference sensor fault");
title("estimated fault vs reference fault");