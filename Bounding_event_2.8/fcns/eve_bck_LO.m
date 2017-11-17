function [zeroCrossing,isterminal,direction,book] = ...
    eve_bck_LO(t,X,params,book,control)
    
    p = params;
    Tst = p.Tst;
    Tsw = control.Tsw;
    
    t_TD_b = book.t_TD_b;
    
%% --- event based bounding ---
    zeroCrossing =  t - t_TD_b - Tst;
    isterminal   =  1;
    direction    =  1;
end