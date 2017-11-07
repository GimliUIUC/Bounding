%% Constants
function [L1,l,M1,J1,g,T_sw,Kpz,Kdz,Kpth,Kdth, ...
    k_time,DeltaT] = control_params_one_link

    L1 = 0.4;   % body length
    l = 0.1;    % leg length
    M1 = 7;
    J1 = 0.8;
    g = 9.81;
    T_sw = 0.624;
    Kpz = 0;
    Kdz = 0;
    Kpth = 0;
    Kdth = 0;
    k_time = 0.1;
    DeltaT = 0.347;

end