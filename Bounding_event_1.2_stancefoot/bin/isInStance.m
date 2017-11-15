function [isInStanceBool, stanceFoot] = ...
    isInStance(t,X,params,book,control)

    s_b = X(7);
    s_f = X(8);
    th = pi/2 + X(3);
    
    %% --- book & control ---
    touchdown_back = book.touchdown_back;
    touchdown_front = book.touchdown_front;
    stanceTime = control.stanceTime;
    
    %%
    p0 = fcn_p0(X,params);
    p1 = fcn_p1(X,params);
    
    toe_b = p0 + rot(th) * gen_p_hip2toe(s_b,control);
    toe_f = p1 + rot(th) * gen_p_hip2toe(s_f,control);
    
    %% --- define stance phase ---
%     if(toe_b(2) <=0 && 0<= t - touchdown_back && t - touchdown_back <= stanceTime)
%         stanceFoot = -1;        % hind stance phase
%     elseif(toe_f(2) <=0 && 0<=t - touchdown_front && t - touchdown_front <= stanceTime)
%         stanceFoot = 1;
%     else
%         stanceFoot = 0;
%     end
%     
%     if(stanceFoot == 0)
%         isInStanceBool = 0;
%     else
%         isInStanceBool = 1;
%     end
    
    
    if(toe_b(2) <= 0 || toe_f(2) <= 0)
        isInStanceBool = 1;
    else
        isInStanceBool = 0;
    end
    
    if(toe_b(2) <= 0)
        stanceFoot = -1;
    elseif(toe_f(2) <= 0)
        stanceFoot = 1;
    else
        stanceFoot = 0;
    end

end