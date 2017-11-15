function ic = impactMapping(x,control)
    [L1,M1,J1,g] = control_params_one_link;
    params = [L1,M1,J1,g];
    foot = control.foot;
    
    %% after impact
    q = x(1:3)';
    dq = x(4:6)';
    D = fcn_D(q,params);
    
    if(foot == 1)
        J = fcn_JP(q,params);
    else
        J = [1,0,0;0,1,0];
    end
    
    Mat = [D -J';J zeros(2,2)];

    % initial condition after impact
    % [D -J';J zeros(2,2)]*[dq_pos;Fe] = [D*dq;zeros(2,1)]
    ic(1:3) = x(1:3);
    dq_pos_Fe = Mat\[D*dq;zeros(2,1)];
    dq_pos = dq_pos_Fe(1:3);
    ic(4:6) = dq_pos';

end