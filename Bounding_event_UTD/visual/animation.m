function [GRF, tau] = animation(t,X,te,Xraw,p,control)

x = X(:,1);
xr = Xraw(:,1);
z = X(:,2);
th = X(:,3);
dx = X(:,4);
dz = X(:,5);
dth = X(:,6);
t_s = X(:,7);
s_f = X(:,8);
s_b = X(:,9);

s_f(s_f>1) = 1;
s_b(s_b>1) = 1;


L = p.L;    l = p.l;    M = p.M;    J = p.J;    g = p.g;    l_leg = p.l_leg;
a = p.a;    
Tst = p.Tst;    
Tsw = control.Tsw;
T = Tst + Tsw;
sw_co1 = p.sw_co1;
sw_co2 = p.sw_co2;

scale = 0.001;      % for plotting GRF

Tst = control.Tst;
alpha_z = control.alpha_z;
alpha_tau = control.alpha_tau;
st_co1 = control.coeff1;
st_co2 = control.coeff2;

% ------- event flags -------
idx_bLO = find(t >= te(1),1);
idx_fTD = find(t >= te(2),1);
idx_fLO = find(t >= te(3),1);
idx_bTD = find(t >= te(4),1);

% ------- draw ---------
nt = length(t);
GRF = zeros(2,nt);
tau = zeros(nt,1);
for ii = 1:nt
    hold on
    grid on
    xlabel('x')
    ylabel('z')

    COM = [x(ii);z(ii)];
    p_hip_f = COM + rot(th(ii))*[L/2;0];
    p_hip_b = COM - rot(th(ii))*[L/2;0];
    
    b_f = bezier(s_f(ii),sw_co1,sw_co2);
    b_b = bezier(s_b(ii),sw_co1,sw_co2);
    
    p_toe_f = p_hip_f + l_leg*[0;-b_f];
    p_toe_b = p_hip_b + l_leg*[0;-b_b];
    % body
    plot([p_hip_f(1) p_hip_b(1)],[p_hip_f(2) p_hip_b(2)],'linewidth',3,'color','k')
    % ground
    plot([-1 1],[0 0])
    % text
    text(0,0.6,['t = ',num2str(t(ii),'%.2f')])
    text(0,0.5,['s_f = ',num2str(s_f(ii),'%.2f')])
    text(0,0.4,['s_b = ',num2str(s_b(ii),'%.2f')])
    
    if ii == idx_bLO
        ;
    elseif ii == idx_fTD
        p_toe_fTD = p_toe_f;
    elseif ii == idx_fLO
        ;
    elseif ii == 1
        p_toe_bTD = p_toe_b;
    else
        ;
    end
    
    if t(ii) >= 0 && t(ii) < te(1)  % back stance
        fleg = [p_hip_f, p_toe_f];
        bleg = [p_hip_b, p_toe_bTD];
        
        s_st = t(ii)/Tst;
        b = bezier(s_st,st_co1,st_co2);
        tau(ii) = -alpha_tau * b;
        Fz = alpha_z*b;
        Fx = (tau(ii) + xr(ii)*Fz)/z(ii);
        GRF(:,ii) = [Fx;Fz];
        
        temp = [p_toe_bTD, p_toe_bTD + GRF(:,ii)*scale];
        plot(temp(1,:),temp(2,:),'b')
        
    elseif t(ii) >= te(2) && t(ii) < te(3)  % front stance
        fleg = [p_hip_f, p_toe_fTD];
        bleg = [p_hip_b, p_toe_b];
        
        s_st = (t(ii) - te(2))/Tst;
        b = bezier(s_st,st_co1,st_co2);
        tau(ii) = alpha_tau * b;
        Fz = alpha_z*b;
        Fx = (tau(ii) + xr(ii)*Fz)/z(ii);
        GRF(:,ii) = [Fx;Fz];
        
        temp = [p_toe_fTD, p_toe_fTD + GRF(:,ii)*scale];
        plot(temp(1,:),temp(2,:),'r')
        
    else
        fleg = [p_hip_f, p_toe_f];
        bleg = [p_hip_b, p_toe_b];
        Fx = 0;
        Fz = 0;
        GRF(:,ii) = [Fx;Fz];
        tau(ii) = 0; 
    end
    
    % legs
    plot(fleg(1,:),fleg(2,:),'r')
    plot(bleg(1,:),bleg(2,:),'b')
    
    
    
    axis([-0.2 1 -0.1 1])
%     axis equal
    
    pause(0.05)
%     pause(1);
    if ii<nt
        clf
    end
end

    
end




