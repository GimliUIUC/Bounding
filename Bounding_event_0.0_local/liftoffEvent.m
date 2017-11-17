function [zeroCrossing,isterminal,direction] = liftoffEvent(t,x,book,control)
    q = x(1:4);

    [L1,M1,J1,g] = control_params_one_link;
    params = [L1,M1,J1,g];
    
    stanceTime = control.stanceTime;
    touchdownTime = book.touchdownTime;
    
    % liftoff occurs when duration reaches 
    % stance time since touchdown
    liftoff = (t - touchdownTime) - stanceTime;
    
    zeroCrossing =  liftoff;
    isterminal   =  1       ;
    direction    =  1       ;