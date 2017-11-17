
function dxdt = unconstrained(t,x)
    [L1,M1,J1,g] = control_params_one_link;
    params = [L1,M1,J1,g];
    
    q = x(1:3);
    dq = x(4:6);
    
    D = fcn_D(q,params);
    C = fcn_C(q,dq,params);
    G = fcn_G(q,params);
    
    % Dynamic equation: D*ddq + C*dq + G = B*u
    % unconstrained: ddq = D\(B*u - C*dq - G)
    
%     Kp = [zeros(2,2) 50*eye(2)];
%     Kd = [zeros(2,2) 5*eye(2)];
%     q_d = [0 0 -pi*2/3 -pi*1/2]';
%     dq_d = [0 0 0 0]';
%     u = -Kp * (q - q_d) - Kd * (dq - dq_d);
%     
%     ddq = D\(B*u - C*dq - G);
    ddq = D\( - C*dq - G);
    dxdt = [dq;ddq];

end