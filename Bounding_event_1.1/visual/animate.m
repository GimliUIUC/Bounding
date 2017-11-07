function [] = animate(t,X,p)

[x, z, th, dx, dz, dth] = decomposeX(X);

L = p.L;    l = p.l;    M = p.M;    J = p.J;    g = p.g;    l_leg = p.l_leg;
Tst = p.Tst;    Tsw = 0.3;
T = Tst + Tsw;

nt = length(t);

for ii = 1:nt
    COM = [x(ii);z(ii)];
    p_hip_f = COM + rot(th(ii))*[L/2;0];
    p_hip_b = COM - rot(th(ii))*[L/2;0];
    plot([p_hip_f(1) p_hip_b(1)],[p_hip_f(2) p_hip_b(2)])
    hold on
    plot([-1 1],[0 0])
    plot([p_hip_f(1) p_hip_f(1)],[p_hip_f(2) p_hip_f(2)-l_leg])
    plot([p_hip_b(1) p_hip_b(1)],[p_hip_b(2) p_hip_b(2)-l_leg])
    grid on
    xlabel('x')
    ylabel('z')
    
    axis([-1 0.5 -0.1 0.5])
    axis equal
    
    pause(0.05)
    clf
end
    
end




