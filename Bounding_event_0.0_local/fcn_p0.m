function [p0] = fcn_p0(q,params)

p0 = zeros(2,1);

  p0(1,1)=q(1) + (params(1)*sin(q(3)))/2;
  p0(2,1)=q(2) - (params(1)*cos(q(3)))/2;

 