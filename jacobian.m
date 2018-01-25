%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Jacobian 
% Mayank Roy
% IIT Delhi
% This program finds the Jacobian of joint velocities w.r.t. end effector
% velocities of a n-DOF Manipulator given it's DH parameters.
%%%%%%%%%%%%%%%%%%%%%%%%

clc;clear all;  

%****Enter number of dofs**** 
n = 3;
fprintf('No of Dofs:%d\n',n);

%Initialize Symbols
A = sym('a',[1,n]);
D = sym('d',[1,n]);
Alpha = sym('alpha',[1,n]);
Theta = sym('theta',[1,n]);


Mul = 1;

for i = 1:n      
        Transd = [1 0 0 0;
                  0 1 0 0;
                  0 0 1 D(i);
                  0 0  0 1];
        Rotz = [cos(Theta(i)) -sin(Theta(i)) 0 0;
                sin(Theta(i))  cos(Theta(i)) 0 0;
                   0            0      1 0;
                   0            0      0 1];
        Transa = [1 0 0 A(i);
                  0 1 0 0;
                  0 0 1 0;
                  0 0 0 1];
        Rotx = [   1  0  0  0;
                  0  cos(Alpha(i)) -sin(Alpha(i)) 0;
                  0  sin(Alpha(i))  cos(Alpha(i)) 0;                  
                  0      0            0     1];    
        A_i= Transd*Rotz*Transa*Rotx;
        Mul = Mul*A_i;
        
end

fprintf('The final Transformation Matrix is: \n');
Mul

%*****Enter DH Parameters*****
Temp = subs(Mul,D,zeros(1,n));
Temp = subs(Temp,Alpha,zeros(1,n));

planar = 1;
generalized_coordinates = [Theta(1),Theta(2),Theta(3)];

if planar
    pnp = 2;
else
    pnp = 3;
end
fprintf('The final position of End effector (x,y,z) in inertial frame:\n');
P = Temp(1:3,4)

for i = 1:n
    for j = 1:pnp
        DP(j,i) = diff(P(j),generalized_coordinates(i));
    end
end


disp('*******Jacobian********')
DP


