function [] = animate(t,X,te,Xe,p)

x = X(:,1);
z = X(:,2);
th = X(:,3);
dx = X(:,4);
dz = X(:,5);
dth = X(:,6);
t_ = X(:,7);

L = p.L;    l = p.l;    M = p.M;    J = p.J;    g = p.g;    l_leg = p.l_leg;
Tst = p.Tst;    Tsw = 0.3;
T = Tst + Tsw;

% ------- event flags -------
flag_bTD = 0;
flag_bLO = 0;
flag_fTD = 0;
flag_fLO = 0;

% ------- draw ---------

nt = length(t);
idx_num = 1;        % idx_num = from 1 to 4
for ii = 1:nt
    hold on
    grid on
    xlabel('x')
    ylabel('z')
    
    flag_bTD = (t(ii) == te(1));
    flag_bLO = (t(ii) == te(2));
    flag_fTD = (t(ii) == te(3));
    flag_fLO = (t(ii) == te(4));
    
    COM = [x(ii);z(ii)];
    p_hip_f = COM + rot(th(ii))*[L/2;0];
    p_hip_b = COM - rot(th(ii))*[L/2;0];
    p_toe_f = p_hip_f + [0;-l_leg];
    p_toe_b = p_hip_b + [0;-l_leg];
    % body
    plot([p_hip_f(1) p_hip_b(1)],[p_hip_f(2) p_hip_b(2)])
    % ground
    plot([-1 1],[0 0])
    
    
    if flag_bTD
        p_toe_bTD = p_toe_b;
    elseif flag_bLO
        ;
    elseif flag_fTD
        p_toe_fTD = p_toe_f;
    elseif flag_fLO
        ;
    else
        ;
    end
    
    if t(ii) >= te(1) && t(ii) < te(2)  % back stance
        fleg = [p_hip_f, p_toe_f];
        bleg = [p_hip_b, p_toe_bTD];
    elseif t(ii) >= te(3) && t(ii) < te(4)  % front stance
        fleg = [p_hip_f, p_toe_fTD];
        bleg = [p_hip_b, p_toe_b];
    else
        fleg = [p_hip_f, p_toe_f];
        bleg = [p_hip_b, p_toe_b];
    end
        
        
    
    plot(fleg(1,:),fleg(2,:))
    plot(bleg(1,:),bleg(2,:))
    
    
    axis([-0.2 1 -0.1 0.5])
%     axis equal
    
    pause(0.05)
    clf
end
    
end




