function [zeroCrossing,isterminal,direction,book] = ...
    eve_bck_TD(t,X,params,book,control)
    
    p = params;
    L = p.L;
    l_leg = p.l_leg;
    
    x = X(1);
    z = X(2);
    th = X(3);
    toe_b_z = z - L/2*sin(th) - l_leg;

%% --- event based bounding ---
    zeroCrossing =  toe_b_z;
    isterminal   =  1;
    direction    =  -1;
end