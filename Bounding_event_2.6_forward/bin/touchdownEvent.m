function [zeroCrossing,isterminal,direction] = ...
    touchdownEvent(t,X,book,control)

    q = X(1:3);
    
    [L1,M1,J1,g] = control_params_one_link;
    params = [L1,M1,J1,g];
    
    liftoffTime = book.liftoffTime;
    flightTime = control.flightTime;
    
    touchdown = t - liftoffTime - flightTime;
    
    zeroCrossing =  touchdown;
    isterminal   =  1        ;
    direction    =  1       ;
    
    foot = control.foot;    % switch touchdown leg