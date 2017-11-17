%% Constants
function p = ctrl_params

    p.L = 0.35;     % body length
    p.l = 0.14;     % leg length
    p.M = 5;
    p.J = 0.3;
    p.g = 9.81;
    p.l_leg = 0.17;
    p.Tst = 0.07;
    p.a = 0;
    
    p.st_co1 = [0 0.8 1 1];
    p.st_co2 = [1 1 0.8 0];
    p.sw_co1 = [1 0.6 0.5 0.5];
    p.sw_co2 = [0.5 0.5 0.6 1];
%     p.sw_co1 = [1 1 1 1];
%     p.sw_co2 = [1 1 1 1];
    
    p.Kdx = 720;
    p.Kpz = 0;
    p.Kdz = 0;
    p.Kpth = 5;
    p.Kdth = 5;
    
    p.Nstep = 6;
    
    p.k_time = 0.2;
    p.DeltaT = 0.347;
    
    

end