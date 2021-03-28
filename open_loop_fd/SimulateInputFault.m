function [y, fault_val] = SimulateInputFault(y, t, type)
YAW_ANGLE_IDX = 3;
FAULT_STARTS = 20;

fault_val = 0;
if type == "stepwise"
    if t > FAULT_STARTS
        fault_val = deg2rad(10);
        
    end
elseif type == "driftwise"
        if t > FAULT_STARTS
            fault_val = deg2rad(0.5*(t-FAULT_STARTS));
        end
end

y(YAW_ANGLE_IDX) = y(YAW_ANGLE_IDX) + fault_val;
end

