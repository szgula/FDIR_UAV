function [y, fault_val] = SimulateInputFault(y, t, type)
YAW_ANGLE_IDX = 3;
FAULT_STARTS = 12.3;

fault_val = 0;
if type == "stepwise"
    if t > FAULT_STARTS
        fault_val = deg2rad(30);
        
    end
elseif type == "driftwise"
        if t > FAULT_STARTS
            fault_val = deg2rad((t-FAULT_STARTS)*2);
        end
end

y(YAW_ANGLE_IDX) = y(YAW_ANGLE_IDX) + fault_val;
end

