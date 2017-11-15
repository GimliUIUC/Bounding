function [zeroCrossing,isterminal,direction,book] = ...
    eve_frt_LO(t,X,params,book,control)
    
    p = params;
    Tst = p.Tst;
    
    t_TD_f = book.t_TD_f;
    
%% --- event based bounding ---
    zeroCrossing =  t - t_TD_f - Tst;
    isterminal   =  1;
    direction    =  1;
end