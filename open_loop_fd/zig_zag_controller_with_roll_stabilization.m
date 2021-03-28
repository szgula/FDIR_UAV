function [output, controller_data] = zig_zag_controller_with_roll_stabilization(system_state, target_heading, controller_data);

%%%%%%%%%%%%%%%%% CONFIGURATION %%%%%%%%%%%%%%%%%
HEADING_IDX = 3;  % yaw angle
ROLL_IDX = 4;  % yaw angle

last_output = controller_data.output;
dt = controller_data.dt;
last_target_heading = controller_data.target_heading;
last_heading = controller_data.heading;
last_roll = controller_data.roll;

output_limit = pi/3;        % 60 deg
output_change_limit = dt * pi/12;   % 15 deg/s 

% for desired_r = 0.1,  K_ailerons_pos > 11. system becomes unstable
% for desired_r = 0.2,  K_ailerons_pos > 10.4 system becomes unstable
% for desired_r = 0.5,  K_ailerons_pos > 10.6 system becomes unstable
% for desired_r = 1,  K_ailerons_pos > 10.6 system becomes unstable
Kp_ailerons_pos = 10.0; 
Ki_ailerons_pos = 2; 
ailerons_rate_limit = 0.5;


%%%%%%%%%%%%%%%%% PRE-CALCULATIONS %%%%%%%%%%%%%%%%%
dHeading = target_heading - system_state(HEADING_IDX);
dRoll = 0 - system_state(ROLL_IDX);
last_dRoll = 0 - last_roll;
derivative_a = (dRoll - last_dRoll) / dt;
last_output_a = last_output(1);
last_output_r = last_output(2);

%%%%%%%%%%%%%%%%% GET DESIRE AILERONS POSITION %%%%%%%%%%%%%%%%%
desired_a = Kp_ailerons_pos * dRoll + derivative_a * Ki_ailerons_pos;
%%%%%%%%%%%%%%%%% GET DESIRE RUDDER POSITION %%%%%%%%%%%%%%%%%
desired_r = sign(dHeading) * output_limit;
%desired_r = 0.1;

%%%%%%%%%%%%%%%%% MOVE OUTPUT TOWARDS THE DEFIRE POSITION %%%%%%%%%%%%%%%%%
da = desired_a - last_output_a;
output_a = last_output_a + sign(da) * output_change_limit * ailerons_rate_limit;

dr = desired_r - last_output_r;
output_r = last_output_r + sign(dr) * output_change_limit * ailerons_rate_limit;


%%%%%%%%%%%%%%%% LIMIT THE OUTPUT %%%%%%%%%%%%%%%%%
output_a = min(output_limit, max(-output_limit, output_a));
output_r = min(output_limit, max(-output_limit, output_r));


%%%%%%%%%%%%%%%% PREPARE OUTPUT %%%%%%%%%%%%%%%%%
output = [output_a, output_r];

controller_data.output = output;
controller_data.target_heading = target_heading;
controller_data.heading = system_state(HEADING_IDX);
controller_data.roll = system_state(ROLL_IDX);
end

