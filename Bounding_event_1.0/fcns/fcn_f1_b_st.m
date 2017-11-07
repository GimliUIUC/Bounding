function [f] = fcn_f1_b_st(q,F,qd,toe,params)

f = zeros(10,1);

  f(1,1)=q(4);
  f(2,1)=q(5);
  f(3,1)=q(6);
  f(4,1)=-(F(2) - params(10)*(q(6) - qd(4)) + toe(1)*(params(8)*(q(5) - qd(3)) - F(1) + params(7)*...
         (q(2) - qd(1))) - params(9)*(q(3) - qd(2) + pi/2))/(params(3)*toe(2));
  f(5,1)=- params(5) - (params(8)*(q(5) - qd(3)) - F(1) + params(7)*(q(2) - qd(1)))/params(3);
  f(6,1)=-(params(10)*(q(6) - qd(4)) - F(2) + params(9)*(q(3) - qd(2) + pi/2))/params(4);
  f(7,1)=0;
  f(8,1)=1/params(6);
  f(9,1)=1;
  f(10,1)=0;

 