function [G] = fcn_G(q,params)

G = zeros(3,1);

  G(1,1)=0;
  G(2,1)=params(3)*params(5);
  G(3,1)=0;

 