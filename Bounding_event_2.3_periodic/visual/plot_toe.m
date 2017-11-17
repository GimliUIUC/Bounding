function [] = plot_toe(X,params,book,control)

q = X(:,1:3);
th = q(:,3) + pi/2;
s_b = X(:,7);
s_f = X(:,8);

p0 = fcn_p0(q,params);
p1 = fcn_p1(q,params);

toe_b = p0 + rot(th) * gen_p_hip2toe(s_b,control);
toe_f = p1 + rot(th) * gen_p_hip2toe(s_f,control);





end