% Inverse 2-link
function inv_2_links
    [L1,L2,M1,M2,J1,J2,g] = control_params_two_link;
    params = [L1,L2,M1,M2,J1,J2,g];
    
    x0 = [-pi/3 -pi/3 0 0];     % initial condition
    tstart = 0;
    tfinal = 10;
    
    L = length(x0);
    tout = tstart;
    xout = x0;
    teout = [];
    xeout = [];
    ieout = [];
    ntlist = [];
    
    % Pd is a structure
    Pd.x = 1.5;       % desired position
    Pd.y = 1;
    Pd.Kp = 40;         % PD control parameters
    Pd.Kd = 10;
    
    %% use inverse kinematics to calculate q_d
    P_d = [Pd.x;Pd.y];
    q0 = x0(1:2);       % initial guess
    r = 0.0001;       % tolerance radius
    q_d = q0';
    P_curr = fcn_P(q_d,params);
    JP = fcn_JP(q_d,params);
    
%     %% Newton method
%     while abs(P_curr - P_d) > r
%         q_d = q_d + inv(JP)*(P_d - P_curr);
%         P_curr = fcn_P(q_d,params);
%         JP = fcn_JP(q_d,params);
%     end
    
    %% Gradient method
    alpha = 0.01;
    while abs(P_curr - P_d) > r
        q_d = q_d + alpha * JP' * (P_d - P_curr);
        P_curr = fcn_P(q_d,params);
        JP = fcn_JP(q_d,params);
    end
    
    
    q_d
    fcn_P(q_d,params)
    
    Pd.qd = q_d;        % desired joint angles
    
    options = odeset('RelTol',1e-5);
    [t,x] = ode45(@dynamics,[tstart tfinal],x0,options,Pd);
    
    plot(t,x)
    legend('q1','q2','dq1','dq2')
    
    
    rot = inline('[cos(th) -sin(th); sin(th) cos(th)]','th');
    
    for i = 1:length(t)
        q1 = x(i,1);
        q2abs = q1 + x(i,2);
        
        p1 = rot(q1)*[0;L1];
        p2 = p1+rot(q2abs)*[0;L2];
        
        
        hold on
        plot(Pd.x,Pd.y,'r*')
        plot([0 p1(1) p2(1)],[0 p1(2) p2(2)])
        
        axis([-1 2 -1 2])
        
        F(i) = getframe;
        clf
        
    end
    

end

function dxdt = dynamics(t,x,Pd)
    [L1,L2,M1,M2,J1,J2,g] = control_params_two_link;
    params = [L1,L2,M1,M2,J1,J2,g];
    
    L = length(x);
    q = x(1:L/2);
    dq = x(L/2+1:L);

    Kp = Pd.Kp;
    Kd = Pd.Kd;
    qd = Pd.qd;
    
    
    D = fcn_D(q,params);
    C = fcn_C(q,dq,params);
    G = fcn_G(q,params);
    B = fcn_B(q,params);
    
    u = Kp*(qd -q) + Kd*(-dq);

    ddq = inv(D)*(B*u - C*dq - G);
    dxdt = [dq;ddq];
end