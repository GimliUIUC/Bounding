%% one link float body dynamics for robot bounding
% Author: Yanran Ding
% last editted: 11/07/2017

addpath basic fcns gen visual

%%
% --- parameters for eom ---
params = ctrl_params;
p = params;
L = p.L;    l = p.l;    M = p.M;    J = p.J;    g = p.g;    
Tst = p.Tst;    Tsw = 0.3;
T = Tst + Tsw;


coeff1 = [0 0.8 1 1];
coeff2 = [1 1 0.8 0];
c = mean(1/2 * coeff1 + 1/2 * coeff2);

% coeff_x = [-0.1 -0.13 -0.1 -0.07  -0.02 0.02  0.07   0.1  0.13 0.1];
% coeff_z = [-0.35 -0.3 -0.2  -0.25 -0.32 -0.32 -0.25 -0.2 -0.3 -0.35];

% --- control variables ---

control.Tst = Tst;
control.Tsw = Tsw;
control.alpha_z = M*g*T/(2*c*Tst);
control.alpha_tau = 15;
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

%X = [x   z  th   dx dz dth t_]
ic = [0.2 0.25 pi/20 0 0  0 0]; % free fall

tout = tstart;
Xout = ic;
teout = [];
Xeout = [];
ieout = [];


%% ------ free fall ---------
options = odeset('Events',@(t,X)eve_bck_TD(t,X,params,book,control));

[t,X,te,Xe,ie] = ode45(@(t,X)dyn_air(t,X,params,book,control),[tstart, tfinal], ic,options);

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


%% -------- back stance ---------
options = odeset('Events',@(t,X)eve_bck_LO(t,X,params,book,control));

[t,X,te,Xe,ie] = ode45(@(t,X)dyn_bck_st(t,X,params,book,control),[tstart, tfinal], Xout(end,:),options);

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


%% --------- reconstruct x ---------
for ii = 1:2:length(teout)
    idx = find(tout==teout(ii));
    delt_x = Xout(idx,1) - Xout(idx-1,1);
    Xout(idx:end,1) = Xout(idx:end,1) - delt_x;
end

%% --------- visualize -----------
[t_even, X_even] = even_sample(tout,Xout,100);

subplot(1,2,1)
plot(tout,Xout(:,1:3))
legend('x','z','th')

subplot(1,2,2)
plot(tout,Xout(:,4:7))
legend('dx','dz','dth','t_s')

figure
animate(t_even,X_even,teout, Xeout, params)
% animate(tout,Xout,teout,Xeout,params)

