%% one link float body dynamics for robot bounding
% Author: Yanran Ding
% last editted: 10/06/2017

addpath basic fcns gen plot dynamic_fcns

%%
% --- parameters for eom ---
p = control_params_one_link;
L1 = p.L1;
l = p.l;
M1 = p.M1;
J1 = p.J1;
g = p.g;
T_sw = p.T_sw;
Kpz = p.Kpz;
Kdz = p.Kdz;
Kpth = p.Kpth;
Kdth = p.Kdth;
k_time = p.k_time;
DeltaT = p.DeltaT;

params = [L1,l,M1,J1,g,T_sw,Kpz,Kdz,Kpth,Kdth,k_time,DeltaT];
[zd, dzd, thd, dthd] = control_qd;
qd = [zd, dzd, thd, dthd];


% --- initial conditions ---
% x =    [z        th          dx      dz      dth     dfront      dhind  alpha_z   alpha_tau]
optVar = [0.45   -pi/2-0.3   2.2521    0.88    0.8872    0.0775    0.0960  200.0000   28.9702];

ic = [0 optVar(1:5) 0.417/0.694 0  0 0.2];

% --- simulation parameters ---
stanceTime = 0.07;
flightTime = 0.277;
T = 2*(stanceTime + flightTime);
% tmax = 3*T;
tmax = T;
coeff1 = [0 0.8 1 1];
coeff2 = [1 1 0.8 0];
c = mean(1/2 * coeff1 + 1/2 * coeff2);

coeff_x = [-0.1 -0.13 -0.1 -0.07  -0.02 0.02  0.07   0.1  0.13 0.1];
coeff_z = [-0.35 -0.3 -0.2  -0.25 -0.32 -0.32 -0.25 -0.2 -0.3 -0.35];

% --- control variables ---
dfront = optVar(6);
dhind = optVar(7);
alpha_tau = optVar(9);

control.stanceTime = stanceTime;
control.flightTime = flightTime;
control.alpha_z = M1*g*T/(2*c*stanceTime);
control.alpha_tau = alpha_tau;
control.dhind = dhind;
control.dfront = dfront;
control.coeff1 = coeff1;
control.coeff2 = coeff2;
control.coeff_x = coeff_x;
control.coeff_z = coeff_z;
control.foot = -1;      % -1 for hind leg, 1 for front leg

% --- bookkeeping ---
book.touchdown_back = flightTime;
book.liftoff_back = flightTime + stanceTime;
book.touchdown_front = 2*flightTime + stanceTime;
book.liftoff_front = 0;

book.touchdownPoint = 0;
book.touchdownList = zeros(1,0);
book.touchdownTime = zeros(1,0);
book.liftoffTime = zeros(1,0);
book.phase = 'Aerial_1';
book.test = 0;

% =============================

[t,X,book] = runSim(@dynamics,@hybridEvent,@handleEvent, ...
    tmax,params,book, ic, control);

% =============================
%%

subplot(2,2,1)
plot(t,X(:,1:3))
legend('x','z','q1')

subplot(2,2,2)
plot(t,X(:,4:6))
legend('dx','dz','dq1')

subplot(2,2,3)
plot(t,X(:,7:8))
legend('s_b','s_f')

subplot(2,2,4)
plot(t,X(:,9:10))
legend('t','T')


figure
F = drawPicture(t,X,params,book,control);

% %% -- videos ---
% v = VideoWriter('new_video.avi');
% open(v)
% writeVideo(v,F)
% close(v)
% 



