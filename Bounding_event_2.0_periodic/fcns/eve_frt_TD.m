function [zeroCrossing,isterminal,direction,book] = ...
    eve_frt_TD(t,X,params,book,control)
    
    p = params;
    L = p.L;
    l_leg = p.l_leg;
    a = p.a;
    
    x = X(1);
    z = X(2);
    th = X(3);
    toe_f_z = z + L/2*sin(th) - l_leg*cos(a);

%% --- event based bounding ---
    zeroCrossing =  toe_f_z;
    isterminal   =  1;
    direction    =  -1;
end