%% one link float body dynamics for robot bounding
% Author: Yanran Ding
% last editted: 11/15/2017

addpath basic fcns gen visual
x_opt = [0.1632   -0.1391   -0.6140    2.0355];

%%
params = ctrl_params;
p = params;
L = p.L;    l = p.l;    M = p.M;    J = p.J;    g = p.g;    l_leg = p.l_leg;
Tst = p.Tst;    
Tsw = 0.4;
T = Tst + Tsw;
Tair = (Tsw - Tst)/2;

Nstep = p.Nstep;

coeff1 = p.st_co1;
coeff2 = p.st_co2;
c = mean(1/2 * coeff1 + 1/2 * coeff2);

%%

th0 = x_opt(1);
x0 = L/2*cos(th0);
z0 = L/2*sin(th0)+l_leg;
dx0 = x_opt(2);
dz0 = x_opt(3);
dth0 = x_opt(4);
s_f0 = Tair/Tsw;
s_b0 = 0;
% X = [x z th dx dz dth t_ph s_f s_b]
ic = [x0 z0 th0 dx0 dz0 dth0 0 s_f0 s_b0];
alpha_tau = 23;

%%
% --- control variables ---
control.Tst = Tst;
control.Tsw = Tsw;
control.alpha_z = M*g*T/(2*c*Tst);
control.alpha_tau = alpha_tau;
control.coeff1 = coeff1;
control.coeff2 = coeff2;
control.zd = 0.18;
control.dzd = 0;

% --- bookkeeping ---
book.t_TD_f = 0;
book.t_LO_f = 0;
book.t_TD_b = 0;
book.t_LO_b = 0;

tstart = 0;
tfinal = 3;


tout = tstart;
Xout = ic;
teout = [];
Xeout = [];
ieout = [];

for nn = 1:Nstep

%% -------- back stance ---------
options = odeset('Events',@(t,X)eve_bck_LO(t,X,params,book,control),'MaxStep',1e-3);

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
options = odeset('Events',@(t,X)eve_frt_TD(t,X,params,book,control),'MaxStep',1e-3);

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
Xout(end,8) = 0;    % end of front swing

book.t_TD_f = te;
% book.toe_f(end+1) = X(end,1) + L/2*cos(X(end,3));

%% -------- front stance ---------
options = odeset('Events',@(t,X)eve_frt_LO(t,X,params,book,control),'MaxStep',1e-3);

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
options = odeset('Events',@(t,X)eve_bck_TD(t,X,params,book,control),'MaxStep',1e-3);

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
Xout(end,9) = 0;            % end of back swing

book.t_TD_b = te;

ic = Xout(end,:);

end


Xraw = Xout;

%% --------- reconstruct x ---------
for ii = 2:2:length(teout)
    idx = find(tout == teout(ii));
    delt_t = tout(idx) - tout(idx - 1);
    delt_x = Xout(idx,1) - Xout(idx-1,1) - Xout(idx,4)*delt_t;
    Xout(idx:end,1) = Xout(idx:end,1) - delt_x;
end

%% --------- visualize -----------
N = 100;
[t_even, X_even] = even_sample(tout,Xout,N);
[t_even, Xraw_even] = even_sample(tout,Xraw,N);

figure
[GRF,tau] = animation_Nstep(t_even,X_even,teout, Xraw_even, params,control);

%%
figure
subplot(2,2,1)
% plot(t_even,tau)
plot(tout,Xout(:,1:3))
legend('x','z','th')

subplot(2,2,2)
plot(tout,Xout(:,4:6))
legend('dx','dz','dth')

subplot(2,2,3)
plot(tout,Xout(:,7:9))
legend('t_s','s_f','s_b')

subplot(2,2,4)
plot(t_even,GRF(1,:))
hold on
plot(t_even,GRF(2,:))
legend('F_x','F_z')





