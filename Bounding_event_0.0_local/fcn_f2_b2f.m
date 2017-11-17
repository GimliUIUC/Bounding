function [f] = fcn_f2_b2f(q,F,toe,params)

f = zeros(10,1);

  f(1,1)=q(4);
  f(2,1)=q(5);
  f(3,1)=q(6);
  f(4,1)=0;
  f(5,1)=-params(5);
  f(6,1)=0;
  f(7,1)=1/params(6);
  f(8,1)=1/params(6);
  f(9,1)=1;
  f(10,1)=0;

 