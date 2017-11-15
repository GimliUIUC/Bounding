%% constrained
function dxdt = constrained(t,x,book,control)
    [L1,M1,J1,g] = control_params_one_link;
    params = [L1,M1,J1,g];
    
    touchdownTime = book.touchdownTime;
    touchdownPoint = book.touchdownPoint;
    stanceTime = control.stanceTime;
    flightTime = control.flightTime;
    T = 2*(stanceTime + flightTime);
    foot = control.foot;
    
    q = x(1:3);
    dq = x(4:6);
    D = fcn_D(q,params);
    C = fcn_C(q,dq,params);
    G = fcn_G(q,params);
    B = fcn_B(q,params);
    p0 = fcn_p0(q,params);
    p1 = fcn_p1(q,params);
    
    % GRF using Bezier polynomial
    s = (t - touchdownTime)/stanceTime;
    coeff1 = [0 0.8 1 1];
    coeff2 = [1 1 0.8 0];
    c = mean(1/2 * coeff1 + 1/2 * coeff2);
    
    b = 0;
    if(0 <= s && s <= 0.5)
        b = polyval_bz(coeff1, s*2);
    elseif(0.5 < s && s <= 1)
        b = polyval_bz(coeff2, s*2-1);
    end

    % magnitude of Fz profile
    alpha_z = M1*g*T/(2*c*stanceTime);
    alpha_tau = control.alpha_tau;
    
    
    if(foot == 1)       % front foot
        % vector that points from COM to p1
        rf = [touchdownPoint;0] - [q(1);q(2)];
        
        % Moment about COM using wedge product
%         tau = rf(1) * Fe(2) - rf(2) * Fe(1);
        tau = alpha_tau * b;
        
        % GRF
        Fz = alpha_z*b;
        Fx = (rf(1)*Fz - tau)/(rf(2));
        Fe = [Fx;Fz];
        
        % update dynamics
        ddq = [Fe(1)/M1 ; Fe(2)/M1 - g ; tau / J1];
        
    elseif(foot == -1)  % hind foot
        % vector that points from COM to p1
        rh = [touchdownPoint;0] - [q(1);q(2)];
        
        % Moment about COM using wedge product
%         tau = rh(1) * Fe(2) - rh(2) * Fe(1);
        tau = - alpha_tau * b;
        
        % GRF
        Fz = alpha_z*b;
        Fx = (rh(1)*Fz-tau)/(rh(2));
        Fe = [Fx;Fz];
        
        % update dynamics
        ddq = [Fe(1)/M1 ; Fe(2)/M1 - g ; tau / J1];
    end
    
    dxdt = [dq;ddq];
end