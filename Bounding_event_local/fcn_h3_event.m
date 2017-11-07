function [h] = fcn_h3_event(q,params)

h = zeros(10,1);

  h(1,1)=q(1);
  h(2,1)=q(2);
  h(3,1)=q(3);
  h(4,1)=q(4);
  h(5,1)=q(5);
  h(6,1)=q(6);
  h(7,1)=q(7);
  h(8,1)=q(8);
  h(9,1)=0;
  h(10,1)=7/100 - params(11)*(q(10) - params(12) + q(9));

 