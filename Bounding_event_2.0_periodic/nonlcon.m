function [C,Ceq] = nonlcon(xopt)

p = ctrl_params;
L = p.L;
l_leg = p.l_leg;


z = xopt(1);
th = xopt(2);
dx = xopt(3);
dz = xopt(4);
dth = xopt(5);

p_toe_f_z = z + L/2*sin(th) - l_leg;
p_toe_b_z = z - L/2*sin(th) - l_leg;

C = [];
C = [C;-p_toe_f_z];
C = [C;-p_toe_b_z];

% e = fcn_onestep(xopt);
Ceq = [];