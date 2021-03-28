function [controller_data] = init_controller(dt)

controller_data.dt = dt;
controller_data.output = [0,0];
controller_data.target_heading = 0;
controller_data.heading = 0;
controller_data.roll = 0;
end

