%% Constants
function p = ctrl_params

    p.L = 0.35;     % body length
    p.l = 0.14;     % leg length
    p.M = 5;
    p.J = 0.3;
    p.g = 9.81;
    p.l_leg = 0.17;
    p.Tst = 0.07;
    
    
    p.T_sw = 0.624;
    p.Kpz = 0;
    p.Kdz = 0;
    p.Kpth = 0;
    p.Kdth = 0;
    p.k_time = 0.1;
    p.DeltaT = 0.347;

end