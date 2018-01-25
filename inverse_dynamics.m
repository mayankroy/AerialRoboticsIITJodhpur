%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inverse Dynamics
% Mayank Roy
% IIT Delhi
% This program calculates required torque for a two link manipulator
% with known physical properties for a given trajectory(pos., vel., acc.)
%%%%%%%%%%%%%%%%%%%%%%%%

clc;clear all; close all;
m1 = 1;
m2 = 1;
l1 = 1; l2 = 1; lc1 = 0.5; lc2 =0.5;
g = 9.18;
Izz1 = (1/12)*m1*l1^2; Izz2 = (1/12)*m2*l2^2; 
syms t
disp('****Trajectory****')
q1_des = ((pi/2)/3)*(t - (3/(2*pi))*sin((2*pi/3)*t))
q2_des = ((pi/2)/3)*(t - (3/(2*pi))*sin((2*pi/3)*t));

disp('****First Derivative****')
dq1_des = diff(q1_des,t)
dq2_des = diff(q2_des,t);

disp('****Second Derivative****')
ddq1_des = diff(dq1_des,t)
ddq2_des = diff(dq2_des,t);

time = 0:0.1:3;
for n = 1:length(time)
    q1(n) = subs(q1_des,t,time(n));
    q2(n) = subs(q2_des,t,time(n));
    dq1(n) = subs(dq1_des,t,time(n));
    dq2(n) = subs(dq2_des,t,time(n));
    ddq1(n) = subs(ddq1_des,t,time(n));
    ddq2(n) = subs(ddq2_des,t,time(n));
    
end

for n = 1:length(time)
    D = [Izz1+Izz2+m1*lc1^2+m2*(l1^2+lc2^2+2*l1*lc2*cos(q2(n))), Izz2+m2*(lc2^2+l1*lc2*cos(q2(n)));
         Izz2+m2*(lc2^2+l1*lc2*cos(q2(n))), Izz2+m2*lc2^2];

    C = [-m2*l1*lc2*sin(q2(n))*dq2(n), -m2*l1*lc2*sin(q2(n))*(dq1(n) + dq2(n));
         -m2*l1*lc2*sin(q2(n))*dq1(n),                  0                               ];

    G = [(m1*lc1+m2*l1)*g*cos(q1(n))+m2*lc2*g*cos(q1(n)+q2(n));
        m2*lc2*g*cos(q1(n)+q2(n))];

    Tau(n,:) = D*[ddq1(n);ddq2(n)]+C*[dq1(n);dq2(n)]+G;

end

figure(1);
hold on;
plot(time,q1,'r');
%plot(q2,'r','.');
plot(time,dq1,'g');
%plot(dq2,'g','.');
plot(time,ddq1,'b');
%plot(ddq2,'b','.');
hold off;
legend('Trajecory','First Derivative','Second Derivative')


figure(2)
hold on;
plot(time,Tau(:,1),'r');
plot(time,Tau(:,2),'g');
legend('Joint-1','Joint-2')
hold off;