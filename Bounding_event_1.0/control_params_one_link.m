%% Constants
function p = control_params_one_link

    p.L1 = 0.4;   % body length
    p.l = 0.1;    % leg length
    p.M1 = 7;
    p.J1 = 0.8;
    p.g = 9.81;
    p.T_sw = 0.624;
    p.Kpz = 0;
    p.Kdz = 0;
    p.Kpth = 0;
    p.Kdth = 0;
    p.k_time = 0.1;
    p.DeltaT = 0.347;

end