% ==============================================================
% Encode hybrid dynamics
% ==============================================================
function dXdt = dynamics(t,X,params,book,control)
% --- parameters for eom ---
% [L1,l,M1,J1,g,T_sw,Kpz,Kdz,Kpth,Kdth,k_time,DeltaT] = unpackParams(params);
[zd,dzd,thd,dthd] = control_qd;
qd = [zd,dzd,thd,dthd];

% --- control parameters ---
stanceTime = control.stanceTime;
alpha_z = control.alpha_z;
alpha_tau = control.alpha_tau;
coeff1 = control.coeff1;
coeff2 = control.coeff2;
foot = control.foot;

% --- bookkeeping ---
touchdown_back =    book.touchdown_back;
touchdown_front =   book.touchdown_front;
% liftoff_back =      book.liftoff_back;
% liftoff_front =     book.liftoff_front;
touchdownPoint =    book.touchdownPoint;

q = X(1:3);
dq = X(4:6);
t_st = X(9);

% --- Stance and aerial phases ---
[isInStanceBool, stanceFoot] = isInStance(t,X,params,book,control);

 if(isInStanceBool)
    %% stance phase
    if(stanceFoot == 1)       % front foot
        % --- front stance complete percentage ---
        s_f_st = (t - touchdown_front)/stanceTime;
%         s_f_st(s_f_st<0) = 0;
%         s_f_st(s_f_st>1) = 1;
        
        if(s_f_st>1 || s_f_st <0)
            disp(['something went wrong s_f_st = ',num2str(s_f_st)]);
        end
        
        b = bezier(s_f_st,coeff1,coeff2);
        
        % vector that points from COM to p1
        toe = [touchdownPoint;0] - [q(1);q(2)];
        tau = alpha_tau * b;
        Fz = alpha_z*b;
        F = [Fz, tau];
        
        % update dynamics
        f = fcn_f3_f_st(X,F,qd,toe,params);

    elseif(stanceFoot == -1)  % hind foot
        s_b_st = (t - touchdown_back)/stanceTime;
        if(s_b_st>1 || s_b_st <0)
            disp(['something went wrong s_b_st = ',num2str(s_b_st)]);
        end
        
        b = bezier(s_b_st,coeff1,coeff2);
        
        % vector that points from COM to p1
        toe = [touchdownPoint;0] - [q(1);q(2)];
        tau = - alpha_tau * b;
        
        % --- GRF ---
        Fz = alpha_z*b;
        F = [Fz;tau];
        
%         fprintf('t = %.3d\t s_b_st = %.3d\t b = %.3d \t Fz = %.3d\t tau = %.3d\b',...
%             t,s_b_st,b,Fz,tau);

        % update dynamics
        f = fcn_f1_b_st(X,F,qd,toe,params);
    end
else
    %% aerial phase
    if(foot == -1)   % back foot
        f = fcn_f4_f2b(X,[],[],params);
        
    elseif(foot == 1)    % front foot
        f = fcn_f2_b2f(X,[],[],params);
        
    end
end

dXdt = f;

end


