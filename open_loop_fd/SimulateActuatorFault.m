function [u, fault_val] = SimulateActuatorFault(u, t, type)
RUDDER_IDX = 2;
FAULT_STARTS = 24.6;
MAX_ACTUATOR_VALUE = deg2rad(60);

fault_val = 0;
if type == "stepwise"
    if t > FAULT_STARTS
        fault_val = deg2rad(50);
    end
elseif type == "driftwise"
        if t > FAULT_STARTS
            fault_val = deg2rad((t-FAULT_STARTS) * 0.56);
        end
end

u(RUDDER_IDX) = u(RUDDER_IDX) + fault_val;
u = min(MAX_ACTUATOR_VALUE, max(u, -MAX_ACTUATOR_VALUE));
end


 