function [D] = fcn_D(q,params)

D = zeros(3,3);

  D(1,1)=params(3);
  D(1,2)=0;
  D(1,3)=0;
  D(2,1)=0;
  D(2,2)=params(3);
  D(2,3)=0;
  D(3,1)=0;
  D(3,2)=0;
  D(3,3)=params(4);

 