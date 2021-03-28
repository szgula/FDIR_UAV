function [FD_internal] = FaultDetection(r_prim, u, FD_internal)

FD_internal.history = [FD_internal.history; r_prim];
FD_internal.r_prim_sum = FD_internal.r_prim_sum + r_prim;
FD_internal.r_prim_sum_hist = [FD_internal.r_prim_sum_hist; FD_internal.r_prim_sum]; 

stat = FD_internal.r_prim_f_status | (abs(r_prim) > FD_internal.r_prim_thresholds);
FD_internal.r_prim_f_status = stat;
FD_internal.r_prim_f_status_history = [FD_internal.r_prim_f_status_history;
                                       FD_internal.r_prim_f_status];
                                   
                                   
end

