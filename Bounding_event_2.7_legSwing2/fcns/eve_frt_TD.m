function [zeroCrossing,isterminal,direction,book] = ...
    eve_frt_TD(t,X,params,book,control)
    
    p = params;
    L = p.L;
    l_leg = p.l_leg;
    a = p.a;
    
    coz1 = p.sw_coz1;
    coz2 = p.sw_coz2;
    
    Tsw = control.Tsw;
    
    x = X(1);
    z = X(2);
    th = X(3);
    s_f = X(8);     % front leg swing completion
    
%% --- leg swing ---
    s_f(s_f>1) = 1;
    bz = bezier(s_f,coz1,coz2);
    
    toe_f_z = z + L/2*sin(th) - l_leg*bz;

%% --- event based bounding ---
    zeroCrossing =  toe_f_z;
    isterminal   =  1;
    direction    =  -1;
end