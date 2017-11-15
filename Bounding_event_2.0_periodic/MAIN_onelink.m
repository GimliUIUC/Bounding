%% one link float body dynamics for robot bounding
% Author: Yanran Ding
% last editted: 11/07/2017

function e = fcn_onestep(xopt)

%%
params = ctrl_params;
p = params;
L = p.L;    l = p.l;    M = p.M;    J = p.J;    g = p.g;    l_leg = p.l_leg;
Tst = p.Tst;    Tsw = 0.4;
T = Tst + Tsw;

%%
xopt = [0.9016    0.0040   -0.4746   -0.5371];

th0 = 0.15;
x0 = L/2*cos(th0);
z0 = L/2*sin(th0)+l_leg;
dx0 = 0;
dz0 = -0.85;
dth0 = pi*0.6;
ic = [x0 z0 th0 dx0 dz0 dth0 0];
th0 = xopt(1);
x0 = L/2*cos(th0);
z0 = L/2*sin(th0)+l_leg;
% ic = [x0 z0 xopt(1:4) 0];
alpha_tau = 22;
%%

coeff1 = [0 0.8 1 1];
coeff2 = [1 1 0.8 0];
c = mean(1/2 * coeff1 + 1/2 * coeff2);

% coeff_x = [-0.1 -0.13 -0.1 -0.07  -0.02 0.02  0.07   0.1  0.13 0.1];
% coeff_z = [-0.35 -0.3 -0.2  -0.25 -0.32 -0.32 -0.25 -0.2 -0.3 -0.35];

% --- control variables ---

control.Tst = Tst;
control.Tsw = Tsw;
control.alpha_z = M*g*T/(2*c*Tst);
control.alpha_tau = alpha_tau;
% control.dhind = dhind;
% control.dfront = dfront;
control.coeff1 = coeff1;
control.coeff2 = coeff2;
% control.coeff_x = coeff_x;
% control.coeff_z = coeff_z;
% control.foot = -1;      % -1 for hind leg, 1 for front leg

% --- bookkeeping ---
book.t_TD_f = 0;
book.t_LO_f = 0;
book.t_TD_b = 0;
book.t_LO_b = 0;

tstart = 0;
tfinal = 10;


tout = tstart;
Xout = ic;
teout = [];
Xeout = [];
ieout = [];


%% -------- back stance ---------
options = odeset('Events',@(t,X)eve_bck_LO(t,X,params,book,control));

[t,X,te,Xe,ie] = ode45(@(t,X)dyn_bck_st(t,X,params,book,control),[tstart, tfinal], ic,options);

nt = length(t);
tout = [tout;t(2:nt)];
Xout = [Xout;X(2:nt,:)];
teout = [teout;te];
Xeout = [Xeout;Xe];
ieout = [ieout;ie];
tstart = tout(end);

Xout(end,7) = 0;

%% --------- aerial phase --------
options = odeset('Events',@(t,X)eve_frt_TD(t,X,params,book,control));

[t,X,te,Xe,ie] = ode45(@(t,X)dyn_air(t,X,params,book,control),[tstart, tfinal], Xout(end,:),options);

nt = length(t);
tout = [tout;t(2:nt)];
Xout = [Xout;X(2:nt,:)];
teout = [teout;te];
Xeout = [Xeout;Xe];
ieout = [ieout;ie];
tstart = tout(end);

Xout(end,1) = -L/2*cos(X(end,3));    % reset x coordinate
Xout(end,7) = 0;

book.t_TD_f = te;
% book.toe_f(end+1) = X(end,1) + L/2*cos(X(end,3));

%% -------- front stance ---------
options = odeset('Events',@(t,X)eve_frt_LO(t,X,params,book,control));

[t,X,te,Xe,ie] = ode45(@(t,X)dyn_frt_st(t,X,params,book,control),[tstart, tfinal], Xout(end,:),options);

nt = length(t);
tout = [tout;t(2:nt)];
Xout = [Xout;X(2:nt,:)];
teout = [teout;te];
Xeout = [Xeout;Xe];
ieout = [ieout;ie];
tstart = tout(end);

Xout(end,7) = 0;

%% ------ second aerial phase ---------
options = odeset('Events',@(t,X)eve_bck_TD(t,X,params,book,control));

[t,X,te,Xe,ie] = ode45(@(t,X)dyn_air(t,X,params,book,control),[tstart, tfinal], Xout(end,:),options);

nt = length(t);
tout = [tout;t(2:nt)];
Xout = [Xout;X(2:nt,:)];
teout = [teout;te];
Xeout = [Xeout;Xe];
ieout = [ieout;ie];
tstart = tout(end);

Xout(end,1) = L/2*cos(X(end,3));    % reset x coordinate
Xout(end,7) = 0;

book.t_TD_b = te;
% book.toe_b(end+1) = X(end,1) - L/2*cos(X(end,3));


%% --------- reconstruct x ---------
for ii = 2:2:length(teout)
    idx = find(tout == teout(ii));
    delt_t = tout(idx) - tout(idx - 1);
    delt_x = Xout(idx,1) - Xout(idx-1,1) - Xout(idx,4)*delt_t;
    Xout(idx:end,1) = Xout(idx:end,1) - delt_x;
end

xdiff = Xout(end,3:6) - xopt(1:4);
% xdiff(2) = xdiff(2);
e = xdiff * xdiff';

end


