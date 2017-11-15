% find ic for periodic orbit
% xopt = [z th dx dz dth]'

addpath basic fcns gen visual

% z0 = 0.2;
th0 = 0.15;
dx0 = 0;
dz0 = -0.85;
dth0 = pi*0.6;
Tsw0 = 0.4;
guess = [th0 dx0 dz0 dth0 Tsw0];

% lbz = 0.1;      ubz = 0.25;
lbth = -pi/3;   ubth = pi/3;
lbdx = 0;       ubdx = 0.3;
lbdz = -4;      ubdz = 4;
lbdth = -pi;    ubdth = pi;
lbTsw = 0.25;   ubTsw = 0.45;


A = [];
b = [];
Aeq = [];
beq = [];
lb = [lbth lbdx lbdz lbdth lbTsw];
ub = [ubth ubdx ubdz ubdth ubTsw];

options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
% options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');
[x,fval, exitflag,output] = fmincon(@fcn_onestep,guess,A,b,Aeq,beq,lb,ub,[],options);










%% Archive
% xopt = sdpvar(5,1);
% lb = [0.21 -pi/3 0 -1 -pi]';
% ub = [0.30  pi/3 3 1 pi]';
% 
% Xend = fcn_onestep(xopt);
% xend = Xend(1:6)';
% 
% e = xopt - xend;
% Obj = e'*e;
% F = [];
% F = [F;(lb <= xopt) && (xopt <= ub)];
% 
% options = sdpsettings('verbose',1,'solver','fmincon');
% sol = optimize(F,Obj,options);
% 
% 
% value(xopt)


