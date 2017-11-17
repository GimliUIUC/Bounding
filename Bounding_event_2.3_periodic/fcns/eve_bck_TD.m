function [zeroCrossing,isterminal,direction,book] = ...
    eve_bck_TD(t,X,params,book,control)
    
    p = params;
    L = p.L;
    l_leg = p.l_leg;
    a = p.a;
    
    Tsw = control.Tsw;
    Tst = control.Tst;
    
    co1 = p.sw_co1;
    co2 = p.sw_co2;

    x = X(1);
    z = X(2);
    th = X(3);
    s_b = X(9);
    
%% --- leg swing ---
    s_b(s_b>1) = 1;
    b = bezier(s_b,co1,co2);
    
    toe_b_z = z - L/2*sin(th) - l_leg*b;

%% --- event based bounding ---
    zeroCrossing =  toe_b_z;
    isterminal   =  1;
    direction    =  -1;
end