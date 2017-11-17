function [L1,l,M1,J1,g,T_sw,Kpz,Kdz,Kpth,Kdth,k_time,DeltaT] = ...
    unpackParams(params)


L1 = params(1); l = params(2); M1 = params(3); J1 = params(4);
g = params(5); T_sw = params(6);
Kpz = params(7);Kdz = params(8);Kpth = params(9);Kdth = params(10);
k_time = params(11);DeltaT = params(12);

end