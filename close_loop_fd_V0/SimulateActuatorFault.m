function [u, fault_val] = SimulateActuatorFault(u, t, type)
RUDDER_IDX = 2;
FAULT_STARTS = 12.3;
MAX_ACTUATOR_VALUE = deg2rad(60);

fault_val = 0;
if type == "stepwise"
    if t > FAULT_STARTS
        fault_val = deg2rad(50);
    end
elseif type == "driftwise"
        if t > FAULT_STARTS
            fault_val = deg2rad((t-FAULT_STARTS) * 2.5);
        end
end

fault_val = min(MAX_ACTUATOR_VALUE, max(fault_val, -MAX_ACTUATOR_VALUE));
u(RUDDER_IDX) = u(RUDDER_IDX) + fault_val;
u = min(MAX_ACTUATOR_VALUE, max(u, -MAX_ACTUATOR_VALUE));
end


 