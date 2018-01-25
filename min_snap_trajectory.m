%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Min. Snap Trajectory
% Mayank Roy
% IIT Delhi
% This program interpolates a seventh degree polynomial - spline,
% to find the minimum snap trajectory.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
clear all;close all;
%input 
initial = [0,0];final = [10,16];

%waypoints
N_way = 3;
Way_p = [3,  4;
        4,6];
Points = [initial;Way_p;final];    

deg = 7;

syms x;

px = [x^7 x^6 x^5  x^4  x^3  x^2  x  1];
dpx = [7*x^6 6*x^5 5*x^4  4*x^3  3*x^2  2*x  1 0];
ddpx = [42*x^5 30*x^4 12*x^3  12*x^2  6*x 2 0 0];
dddpx  = [210*x^4 120*x^3 36*x^2  24*x  6 0 0 0];
ddddpx = [840*x^3 360*x^2 72*x  24  0 0 0 0];
dddddpx = [2520*x^2 720*x 72  0  0 0 0 0];
ddddddpx = [5040*x 720 0  0  0 0 0 0];

%for i = size(Way_p,2)+2    
    
T = [   
        subs(px,x,Points(1,1)), zeros(1,16);
        subs(px,x,Points(2,1)),zeros(1,16);
        zeros(1,8),subs(px,x,Points(2,1)),zeros(1,8);
        zeros(1,8),subs(px,x,Points(3,1)),zeros(1,8);
        zeros(1,16),subs(px,x,Points(3,1));
        zeros(1,16),subs(px,x,Points(4,1));
        subs(dpx,x,Points(1,1)), zeros(1,16);
        zeros(1,16), subs(dpx,x,Points(4,1));
        subs(ddpx,x,Points(1,1)), zeros(1,16);
        zeros(1,16), subs(ddpx,x,Points(4,1));
        subs(dddpx,x,Points(1,1)), zeros(1,16);
        zeros(1,16), subs(dddpx,x,Points(4,1));
        subs(ddddpx,x,Points(1,1)), zeros(1,16);
        zeros(1,16), subs(ddddpx,x,Points(4,1));
        subs(dddddpx,x,Points(1,1)), zeros(1,16);
        zeros(1,16), subs(dddddpx,x,Points(4,1));
        subs(ddddddpx,x,Points(1,1)), zeros(1,16);
        zeros(1,16), subs(ddddddpx,x,Points(4,1));
        subs(dpx,x,Points(2,1)),-subs(dpx,x,Points(2,1)), zeros(1,8);
        subs(ddpx,x,Points(2,1)),-subs(ddpx,x,Points(2,1)), zeros(1,8);
        subs(dddpx,x,Points(2,1)),-subs(dddpx,x,Points(2,1)), zeros(1,8);
        %subs(ddddpx,x,Points(2,1)),-subs(ddddpx,x,Points(2,1)), zeros(1,8);
        zeros(1,8), subs(dpx,x,Points(3,1)),-subs(dpx,x,Points(3,1));
        zeros(1,8), subs(ddpx,x,Points(3,1)),-subs(ddpx,x,Points(3,1));
        zeros(1,8), subs(dddpx,x,Points(3,1)),-subs(dddpx,x,Points(3,1));
        %zeros(1,8), subs(ddddpx,x,Points(3,1)),-subs(ddddpx,x,Points(3,1));
     ];

Y = [Points(1,2);Points(2,2);Points(2,2);Points(3,2);Points(3,2);Points(4,2);0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0];

C = double(T\Y);
a = C([1:8]);
b = C([9:16]);
c = C([17:24]);

p  = [a(1)*x^7 + a(2)* x^6 + a(3) * x^5 + a(4) * x^4 + a(5) * x^3 + a(6) * x^2 + a(7) * x + a(8) * 1;
     b(1)*x^7 + b(2)* x^6 + b(3) * x^5 + b(4) * x^4 + b(5) * x^3 + b(6) * x^2 + b(7) * x + b(8) * 1;
     c(1)*x^7 + c(2)* x^6 + c(3) * x^5 + c(4) * x^4 + c(5) * x^3 + c(6) * x^2 + c(7) * x + c(8) * 1;];

hold on;
%plot(link(3,1),link(3,2),'r.','MarkerSize',30);
    axis([-20 20 -20 20]);
for i = 1:3
    plot(Points(i,1),Points(i,2),'b.','MarkerSize',30);
    for t = Points(i,1) : 0.01 : Points(i+1,1);
        plot(t,subs(p(i),x,t),'r');
    end
end
plot(Points(4,1),Points(4,2),'b.','MarkerSize',30);
