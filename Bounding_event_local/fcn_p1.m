function [p1] = fcn_p1(q,params)

p1 = zeros(2,1);

  p1(1,1)=q(1) - (params(1)*sin(q(3)))/2;
  p1(2,1)=q(2) + (params(1)*cos(q(3)))/2;

 