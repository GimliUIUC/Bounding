clear all;
%% --- define symbols ---
% ===== constants =====
syms L1 l M1 J1 g real
syms Kpz Kdz Kpth Kdth real
% ===== state variables =====
syms x z q1 real
syms dx dz dq1 real
% ===== desired states =====
syms zd thd dzd dthd real
% ===== others =====
syms Fz_ tau_ real          % nominal force values
syms toe_x toe_z real       % toe positions in world frame
syms s_b s_f t T real       % swing phase completion and time variables
syms T_swing real           % swing time
syms k_time DeltaT real

%% --- variable and parameter lists ---
m_list_params = {
    'L1' 'params(1)';
    'l' 'params(2)';
    'M1' 'params(3)';
    'J1' 'params(4)';
    'g' 'params(5)';
    'T_swing' 'params(6)';
    'Kpz' 'params(7)';
    'Kdz' 'params(8)';
    'Kpth' 'params(9)';
    'Kdth' 'params(10)';
    'k_time' 'params(15)';
    'DeltaT' 'params(16)'};

m_list_q = {
    'x','q(1)';
    'z','q(2)';
    'q1','q(3)';
    'dx','q(4)';
    'dz','q(5)';
    'dq1','q(6)';
    's_b','q(7)';
    's_f','q(8)';
    't','q(9)';
    'T','q(10)';};

m_list_qd = {
    'zd' 'qd(1)';
    'thd' 'qd(2)';
    'dzd' 'qd(3)';
    'dthd' 'qd(4)'};

m_list_toe = {
    'toe_x' 'toe(1)';
    'toe_z','toe(2)'};

m_list_Force = {        % nominal force value
    'Fz_','F(1)';
    'tau_','F(2)'};

%% --- variables ---
q = [x z q1]';
dq = [dx dz dq1]';
s = [s_b s_f]';
X_t = [q;dq;s;t;T];
r = [toe_x toe_z];
th = q1 + pi/2;
dth = dq1;
%% --- event h1 back touchdown ---
T_st = 0.07;
h(1,1) = x;
h(2,1) = z;
h(3,1) = q1;
h(4,1) = dx;
h(5,1) = dz;
h(6,1) = dq1;
h(7,1) = s_b;
h(8,1) = s_f;
h(9,1) = 0;
h(10,1) = T_st - k_time*(T + t - DeltaT);

write_fcn_m('fcn_h1_event.m',{'q','params'},...
    [m_list_q;
    m_list_params],{h,'h'});

%% --- dynamics 1: back stance phase ---
% x z q1 dx dz dq1 s_b s_f t T (10)
% --- PD controller for height and pitch ---
Fz = Fz_ - Kpz*(z - zd) - Kdz*(dz - dzd);
tau = tau_ - Kpth*(th - thd) - Kdth*(dth - dthd);

f(1,1) = dx;
f(2,1) = dz;
f(3,1) = dq1;
f(4,1) = (r(1) * Fz - tau)/(r(2) * M1);
f(5,1) = Fz/M1 - g;
f(6,1) = tau/J1;
f(7,1) = 0;
f(8,1) = 1/T_swing;
f(9,1) = 1;
f(10,1) = 0;

write_fcn_m('fcn_f1_b_st.m',{'q','F','qd','toe','params'},...
    [m_list_q;
    m_list_Force;
    m_list_qd;
    m_list_toe;
    m_list_params],{f,'f'});

% f_x = simplify(jacobian(f,X));      % sensitivity test
% write_fcn_m('fcn_f_x_1.m',{'q','F','qd','params'},[m_list_q;m_list_Force;m_list_qd;m_list_params],{f_x,'f_x'});


%% --- event h2 back liftoff ---

h(1,1) = x;
h(2,1) = z;
h(3,1) = q1;
h(4,1) = dx;
h(5,1) = dz;
h(6,1) = dq1;
h(7,1) = 0;
h(8,1) = s_f;
h(9,1) = 0;
h(10,1) = t;

write_fcn_m('fcn_h2_event.m',{'q','params'},...
    [m_list_q;
    m_list_params],{h,'h'});

%% --- dynamics 2: aerial phase b2f ---
% x z q1 dx dz dq1 s_b s_f t T (10)
f(1,1) = dx;
f(2,1) = dz;
f(3,1) = dq1;
f(4,1) = 0;
f(5,1) = - g;
f(6,1) = 0;
f(7,1) = 1/T_swing;
f(8,1) = 1/T_swing;
f(9,1) = 1;
f(10,1) = 0;


write_fcn_m('fcn_f2_b2f.m',{'q','F','toe','params'},...
    [m_list_q;
    m_list_Force;
    m_list_toe;
    m_list_params],{f,'f'});

%% --- event h3 front touchdown ---

h(1,1) = x;
h(2,1) = z;
h(3,1) = q1;
h(4,1) = dx;
h(5,1) = dz;
h(6,1) = dq1;
h(7,1) = s_b;
h(8,1) = s_f;
h(9,1) = 0;
h(10,1) = T_st - k_time*(T+t-DeltaT);

write_fcn_m('fcn_h3_event.m',{'q','params'},...
    [m_list_q;
    m_list_params],{h,'h'});


%% --- dynamics 3: front stance phase ---
% x z q1 dx dz dq1 s_b s_f t T (10)
% --- PD controller for height and pitch ---
Fz = Fz_ - Kpz*(z - zd) - Kdz*(dz - dzd);
tau = tau_ - Kpth*(th - thd) - Kdth*(dth - dthd);

f(1,1) = dx;
f(2,1) = dz;
f(3,1) = dq1;
f(4,1) = (r(1) * Fz - tau)/(r(2) * M1);
f(5,1) = Fz/M1 - g;
f(6,1) = tau/J1;
f(7,1) = 1/T_swing;
f(8,1) = 0;
f(9,1) = 1;
f(10,1) = 0;

write_fcn_m('fcn_f3_f_st.m',{'q','F','qd','toe','params'},...
    [m_list_q;
    m_list_Force;
    m_list_qd;
    m_list_toe;
    m_list_params],{f,'f'});

%% --- event h4 front liftoff ---

h(1,1) = x;
h(2,1) = z;
h(3,1) = q1;
h(4,1) = dx;
h(5,1) = dz;
h(6,1) = dq1;
h(7,1) = s_b;
h(8,1) = 0;
h(9,1) = 0;
h(10,1) = t;

write_fcn_m('fcn_h4_event.m',{'q','params'},...
    [m_list_q;
    m_list_params],{h,'h'});

%% --- dynamics 4: aerial phase f2b ---
% x z q1 dx dz dq1 s_b s_f t T (10)
f(1,1) = dx;
f(2,1) = dz;
f(3,1) = dq1;
f(4,1) = 0;
f(5,1) = -g;
f(6,1) = 0;
f(7,1) = 1/T_swing;
f(8,1) = 1/T_swing;
f(9,1) = 1;
f(10,1) = 0;

write_fcn_m('fcn_f4_f2b.m',{'q','F','toe','params'},...
    [m_list_q;
    m_list_Force;
    m_list_toe;
    m_list_params],{f,'f'});



