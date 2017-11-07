function [zeroCrossing,isterminal,direction,book] = ...
    hybridEvent(t,X,params,book,control)
    
    s_b = X(7);
    s_f = X(7);
    
    th = pi/2 + X(3);        % body pitch angle
    p0 = fcn_p0(X,params);      % hind shoulder position
    p1 = fcn_p1(X,params);      % front shoulder position

    touchdown_back =    book.touchdown_back;
    touchdown_front =   book.touchdown_front;
    stanceTime = control.stanceTime;
    
    %% --- safety net ---
%     s_b(s_b>1) = 1;
%     s_f(s_f>1) = 1;
%     s_b(s_b<0) = 0;
%     s_f(s_f<0) = 0;
%     if(s_b>1 || s_b <0)
%         fprintf('shit something went wrong s_b = %d',s_b)
%     elseif(s_f>1 || s_f<0)
%         fprintf('shit something went wrong s_f = %d',s_f)
%     end
    
    toe_b = p0 + rot(th) * gen_p_hip2toe(s_b,control);
    toe_f = p1 + rot(th) * gen_p_hip2toe(s_f,control);
    

%% --- event based bounding ---
%     safety_s = [100*(0.7-s_b);100*(0.7-s_f)];
%     safety_s(safety_s<0) = 0;
%     
%     backTouchdown =   0*safety_s(1)   + toe_b(2);
%     frontTouchdown =  0*safety_s(2)   + toe_f(2);
    
    backTouchdown = toe_b(2);
    frontTouchdown = toe_f(2);

    backLiftoff =       t - touchdown_back - stanceTime;
    frontLiftoff =      t - touchdown_front - stanceTime;

    zeroCrossing =  [frontTouchdown,frontLiftoff,backTouchdown,backLiftoff];
    isterminal   =  [1             ,1           ,1            ,1          ];
    direction    =  [-1            ,1           ,-1           ,1          ];


end