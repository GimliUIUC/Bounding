function [GRF, tau] = animation_oneFrame(t,X,te,Xraw,p,control,nFrame)

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
Nstep = p.Nstep;
Tst = p.Tst;    
Tsw = control.Tsw;
T = Tst + Tsw;
sw_cox1 = p.sw_cox1;
sw_cox2 = p.sw_cox2;
sw_coz1 = p.sw_coz1;
sw_coz2 = p.sw_coz2;

scale = 0.001;      % for plotting GRF

% stride = Tst*control.dxd;
stride = p.stride;

Tst = control.Tst;
alpha_z = control.alpha_z;
alpha_tau = control.alpha_tau;
st_co1 = control.coeff1;
st_co2 = control.coeff2;

% ------- event flags -------
nt = length(t);
te(2:end+1) = te;
te(1) = 0;
for nn = 1:Nstep
    idx_bTD(nn) = find(t >= te(1+4*(nn-1)),1);
    idx_bLO(nn) = find(t >= te(2+4*(nn-1)),1);
    idx_fTD(nn) = find(t >= te(3+4*(nn-1)),1);
    idx_fLO(nn) = find(t >= te(4+4*(nn-1)),1);
end
idx_bTD(end+1) = nt;
% ------- draw ---------
GRF = zeros(2,nt);
tau = zeros(nt,1);
stepCount = 1;
for ii = 1:nFrame
    hold on
    grid on
    xlabel('x')
    ylabel('z')

    COM = [x(ii);z(ii)];
    p_hip_f = COM + rot(th(ii))*[L/2;0];
    p_hip_b = COM - rot(th(ii))*[L/2;0];
    
    bx_f = bezier(s_f(ii),sw_cox1,sw_cox2);
    bx_b = bezier(s_b(ii),sw_cox1,sw_cox2);
    bz_f = bezier(s_f(ii),sw_coz1,sw_coz2);
    bz_b = bezier(s_b(ii),sw_coz1,sw_coz2);
    
    p_toe_f = p_hip_f + [bx_f*stride/2;-bz_f*l_leg];
    p_toe_b = p_hip_b + [bx_b*stride/2;-bz_b*l_leg];
    % body
    plot([p_hip_f(1) p_hip_b(1)],[p_hip_f(2) p_hip_b(2)],'linewidth',3,'color','k')
    % ground
    plot([-1 6],[0 0])
    % text
    text(0,0.6,['t = ',num2str(t(ii),'%.2f')])
    text(0,0.5,['s_f = ',num2str(s_f(ii),'%.2f')])
    text(0,0.4,['s_b = ',num2str(s_b(ii),'%.2f')])
    
    if ii == idx_bTD(stepCount)
        p_toe_bTD = [p_toe_b(1);0];
    elseif ii == idx_fTD(stepCount)
        p_toe_fTD = [p_toe_f(1);0];
    elseif ii >= idx_bTD(stepCount+1) - 1
        stepCount = stepCount + 1;
        if stepCount > Nstep
            stepCount = Nstep;
        end
    end
    
    if (t(ii) >= te(4*stepCount-3) && t(ii) < te(4*stepCount-2)) % back stance
        fleg = [p_hip_f, p_toe_f];
        bleg = [p_hip_b, p_toe_bTD];
        
        s_st = (t(ii) - te(4*stepCount-3))/Tst;
        b = bezier(s_st,st_co1,st_co2);
        tau(ii) = -alpha_tau * b;
        Fz = alpha_z*b;
        Fx = (tau(ii) + xr(ii)*Fz)/z(ii);
        GRF(:,ii) = [Fx;Fz];
        
        temp = [p_toe_bTD, p_toe_bTD + GRF(:,ii)*scale];
        plot(temp(1,:),temp(2,:),'b')
        
    elseif t(ii) >= te(4*stepCount-1) && t(ii) < te(4*stepCount) % front stance
        fleg = [p_hip_f, p_toe_fTD];
        bleg = [p_hip_b, p_toe_b];
        
        s_st = (t(ii) - te(4*stepCount - 1))/Tst;
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
    
    axis([-0.2 6 -0.5 1])
    hfig = figure(1);
    set(hfig,'Position',[0,300,1400,400]);
    
    
%     axis equal
    
    pause(0.00001)
%     pause(1);
    if ii<nFrame
        clf
    end
end

    
end




