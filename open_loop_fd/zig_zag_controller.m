function [output] = zig_zag_controller(system_state, target_heading, last_output, dt);

HEADING_IDX = 3;  % yaw angle
output_limit = pi/3;        % 60 deg
output_change_limit = dt * pi/12;   % 15 deg/s 

dHeading = target_heading - system_state(HEADING_IDX);
last_output_a = last_output(1);
last_output_r = last_output(2);


desired_a = 0; 
if dHeading > 0
    desired_r = output_limit;
elseif dHeading < 0
    desired_r = -output_limit;
else
    desired_r = 0;
end

da = desired_a - last_output_a;
output_a = last_output_a + sign(da) * output_change_limit;
dr = desired_r - last_output_r;
output_r = last_output_r + sign(dr) * output_change_limit;

output_a = min(output_limit, max(-output_limit, output_a));
output_r = min(output_limit, max(-output_limit, output_r));

output = [output_a, output_r];
end

