function [] = animate(t,X,te,Xe,p)

x = X(:,1);
z = X(:,2);
th = X(:,3);
dx = X(:,4);
dz = X(:,5);
dth = X(:,6);
% t_s = X(:,7);

L = p.L;    l = p.l;    M = p.M;    J = p.J;    g = p.g;    l_leg = p.l_leg;
Tst = p.Tst;    Tsw = 0.3;
T = Tst + Tsw;

% ------- event flags -------
idx_bTD = find(t >= te(1),1);
idx_bLO = find(t >= te(2),1);
idx_fTD = find(t >= te(3),1);
idx_fLO = find(t >= te(4),1);

% ------- draw ---------
nt = length(t);
for ii = 1:nt
    hold on
    grid on
    xlabel('x')
    ylabel('z')

    COM = [x(ii);z(ii)];
    p_hip_f = COM + rot(th(ii))*[L/2;0];
    p_hip_b = COM - rot(th(ii))*[L/2;0];
    p_toe_f = p_hip_f + [0;-l_leg];
    p_toe_b = p_hip_b + [0;-l_leg];
    % body
    plot([p_hip_f(1) p_hip_b(1)],[p_hip_f(2) p_hip_b(2)],'linewidth',3,'color','k')
    % ground
    plot([-1 1],[0 0])
    
    
    if ii == idx_bTD
        p_toe_bTD = p_toe_b;
    elseif ii == idx_bLO
        ;
    elseif ii == idx_fTD
        p_toe_fTD = p_toe_f;
    elseif ii == idx_fLO
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

    plot(fleg(1,:),fleg(2,:),'r')
    plot(bleg(1,:),bleg(2,:),'b')
    
    
    axis([-0.2 1 -0.1 0.5])
%     axis equal
    
    pause(0.05)
    clf
end
    
end




