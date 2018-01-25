%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Visual Servoing
% Mayank Roy
% IIT Delhi
% This program exhibits Visual Servoing using Inverse Kinematics 
% on Initial and Final Point. Then applying Proportional Control with 
% Lyapunov Stability Criterion or exponential decay.
%%%%%%%%%%%%%%%%%%%%%%%%

clc;clear all; close all;
a1 = 1;a2 = 1; a3 =0;
%given circle center  = (0,0) , radius = 4,psi = pi/4;
%theta = 0:0.1:2*pi;
r = 0.8;
psi = -pi/4;

p = [1; 0; 1; 1];
syms A B C px py pz
R_z = [cos(A),-sin(A),0; 
          sin(A),cos(A),0;
             0         0      1];
R_y = [cos(B),0, sin(B);
              0       1      0;
          -sin(B), 0, cos(B)];
      
R_x = [1,     0          0;     
         0, cos(C), -sin(C);
         0, sin(C),  cos(C)];         
      
R = R_z*R_y*R_x;

T = [px;py;pz];

Q = [R T;zeros(1,3),1];
         
fx = 0.5;
fy = 0.5;
tu = 0;
tv = 0;

CI = [fx 0 tu; 0 fy tv; 0 0 1];

p1 = [1 1 0];
p2 = [1 0 0];
q= [p1;p2];

for i = 1:size(q,1)
    Wx = q(i,1);Wy = q(i,2);
    theta2(i) = acos((Wx^2 + Wy^2 - a1^2 - a2^2)/(2*a1*a2));

    A1 = a1+a2*cos(theta2);
    A2 = a2*sin(theta2);

    stheta1 = (Wy*(a1 +a2*cos(theta2)) - Wx * a2 * sin(theta2))/(a1^2 + a2^2 + 2*a1*a2*cos(theta2));
    ctheta1 = (Wx*(a1 +a2*cos(theta2)) + Wy * a2 * sin(theta2))/(a1^2 + a2^2 + 2*a1*a2*cos(theta2));
    theta1(i) = atan2(stheta1,ctheta1);        
    a = theta1(i)+theta2(i);
    b=0;
    c=0;
    p_x = Wx;
    p_y = Wy;
    p_z = 0;

    CE = subs(Q,[A,B,C,px,py,pz],[a,b,c,p_x,p_y,p_z]);
    CE = double(CE);
    tp = CE \ p;
    pp(1) = tp(1)/tp(3);
    pp(2) = tp(2)/tp(3);            
    pp(3) = 1;
    ip(:,i) = CI * pp';

end
dt = 0.01;k = 5;T = 1;
u1 = ip(1,1);v1 = ip(2,1);u2 = ip(1,2);v2 = ip(2,2);
s = [u1;v1];
s_d = [u2;v2];

figure(1);
    
for t = 0:dt:T
    
    L = [-fx/1 0 s(1);0 -(fy/1) -s(2)];
    e = s - s_d;
    J_c = [a1*sin(theta2(1)) 0;a1*cos(theta2(1))+a2 a2;1 1];
    L_m = L*J_c;
    dtheta = -k*inv(L_m)*e;
    theta1(1) = theta1(1) + dtheta(1)*dt;
    theta2(1) = theta2(1) + dtheta(2)*dt;
    
    link = [a1*cos(theta1(1)),a1*sin(theta1(1)); a1*cos(theta1(1))+a2*cos(theta1(1)+theta2(1)),a1*sin(theta1(1))+a2*sin(theta1(1)+theta2(1))];          
           
    a = theta1(1)+theta2(1);
    b=0;
    c=0;
    p_x = link(2,1);
    p_y = link(2,2);
    p_z = 0;

    CE = subs(Q,[A,B,C,px,py,pz],[a,b,c,p_x,p_y,p_z]);
    CE = double(CE);
    tp = CE \ p;
    pp(1) = tp(1)/tp(3);
    pp(2) = tp(2)/tp(3);            
    pp(3) = 1;
    fp = CI * pp';
    s = fp([1,2]);
    
    
    plot(p(1),p(2),'y.','MarkerSize',30);
    hold on;
    plot([0 link(1,1)],[0 link(1,2)],'b-','LineWidth',2);
    plot(link(1,1),link(1,2),'b.','MarkerSize',30);
  
    plot([link(1,1) link(2,1)],[link(1,2) link(2,2)],'b-','LineWidth',2);
    plot( link(2,1),link(2,2),'r.','MarkerSize',30);
   
    axis([-4 4 -4 4]);
    plot(Wx,Wy);
    pause(0.01)
    hold off;    

end


