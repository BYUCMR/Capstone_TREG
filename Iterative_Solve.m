function [ Out ] = Iterative_Solve( params, L12, L23, L31 , d)
%Find the internal angles and kinematic edge lengths from the nominal edge
%lengths.  d is the gap between the rollers

th1=params(1);
th2=params(2);
th3=params(3);

a1=(pi-th1)/2;
a2=(pi-th2)/2;
a3=(pi-th3)/2;

e1=d/sin(th1)*sin(a1);
e2=d/sin(th2)*sin(a2);
e3=d/sin(th3)*sin(a3);

eq(1)=(th1+th2+th3-pi);
eq(2)=sin(th1)/(L23+e2+e3)-sin(th2)/(L31+e3+e1);
eq(3)=sin(th1)/(L23+e2+e3)-sin(th3)/(L12+e2+e1);
% eq(3)=sin(th1)/(L23+e2+e3)-sin(th2)/(L31+e3+e1);

Out=eq*eq'; %Compute the output
end

