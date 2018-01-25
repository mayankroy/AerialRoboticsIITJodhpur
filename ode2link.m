%This is secondary file. It has differential euations
% This file is written by Suril Shah
%Created on 10/04/2014

function dy = ode2link(t,y)
disp(t)
%Stae variables
th1=y(1);
dth1=y(2);
th2=y(3);
dth2=y(4);

%Model parameters
m1 = 1;
m2 = 1;
l1 = 1; l2 = 1; lc1 = 0.5; lc2 =0.5;
g = 9.18;
Izz1 = (1/12)*m1*l1^2; Izz2 = (1/12)*m2*l2^2; 

% Desired values
th_d1=pi/2;dth_d1=0;
th_d2=pi/2;dth_d2=0;
% Gains for PD control
%Free simulation
kp1=700;kd1=70;
% %Force Simulation
kp2=700;kd2=70;

%PID Torque
tu1=-kp1*(th1-th_d1)-kd1*(dth1-dth_d1);
tu2=-kp2*(th2-th_d2)-kd2*(dth2-dth_d2);


%Computed Torque Controller
% ddth_d1 = 0;ddth_d2 = 0;
% tu1_bar = ddth_d1- kp1*(th1 - th_d1) - kd1*(dth1 - dth_d1);
% tu2_bar = ddth_d2- kp2*(th2 - th_d2) - kd2*(dth2 - dth_d2);


% Joint acceleratoin (differential equation)
%ddth=(1/I)*(tu-m*g*(l/2)*cos(th));
D = [Izz1+Izz2+m1*lc1^2+m2*(l1^2+lc2^2+2*l1*lc2*cos(th2)), Izz2+m2*(lc2^2+l1*lc2*cos(th2));
         Izz2+m2*(lc2^2+l1*lc2*cos(th2)), Izz2+m2*lc2^2];

C = [-m2*l1*lc2*sin(th2)*dth2, -m2*l1*lc2*sin(th2)*(dth1 + dth2);
        m2*l1*lc2*sin(th2)*dth1,                  0          ];

G = [(m1*lc1+m2*l1)*g*cos(th1)+m2*lc2*g*cos(th1+th2);
        m2*lc2*g*cos(th1+th2)];

%Computed Torque Controller
% tu = D*[tu1_bar;tu2_bar]+C*[dth1;dth2]+G;
% tu1 = tu(1);tu2 = tu(2);

%tu2 = D*tu2_bar+(C*dth2+G);

%Acceleration Equation
[ddth] = inv(D)*([tu1;tu2]-C*[dth1;dth2]-G);

%ODE 
dy = zeros(4,1);    % a column vector
dy(1) = dth1;
dy(2) = ddth(1);
dy(3) = dth2;
dy(4) = ddth(2);
