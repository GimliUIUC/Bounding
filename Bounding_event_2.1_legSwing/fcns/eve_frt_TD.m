function [zeroCrossing,isterminal,direction,book] = ...
    eve_frt_TD(t,X,params,book,control)
    
    p = params;
    L = p.L;
    l_leg = p.l_leg;
    a = p.a;
    
    co1 = p.sw_co1;
    co2 = p.sw_co2;
    
    Tsw = control.Tsw;
    
    x = X(1);
    z = X(2);
    th = X(3);
    s_f = X(8);     % front leg swing completion
    
%% --- leg swing ---
    s_f(s_f>1) = 1;
    b = bezier(s_f,co1,co2);
    
    toe_f_z = z + L/2*sin(th) - l_leg*b;

%% --- event based bounding ---
    zeroCrossing =  toe_f_z;
    isterminal   =  1;
    direction    =  -1;
end