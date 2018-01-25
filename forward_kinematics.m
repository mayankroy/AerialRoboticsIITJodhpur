%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Forward Kinematics 
% Mayank Roy
% IIT Delhi
% This program finds the end effector position and pose of
% an N-Link Manipulator given the DH - Parameters.
%%%%%%%%%%%%%%%%%%%%%%%%

clc;clear all;  

%Enter number of dofs 
n = 6;
%Enter DH parameters as a mat file
%load 'dh_mat.mat'
%or define matrix
dh_mat = [0.4,pi/6,0, pi/2;
           0, pi/6,0.4,0;
           0 pi/3 0 pi/2;
           0.4 pi/4 0 pi/2;
           0 pi/3,0,pi/2;
           0.4,pi/4,0,0];
%dh_mat = [13,pi/6,0,pi/2;0 pi/6 8 0;-2 pi/3 0 pi/2;8 pi/4 0 -pi/2;0 pi/3,0,pi/2;1,pi/4,0,0]; %puma260
Mul = 1;
for i = 1:n    
        DH = dh_mat(i,:);    
        Transd = [1 0 0 0;
                  0 1 0 0;
                  0 0 1 DH(1);
                  0 0  0 1];
        Rotz = [cos(DH(2)) -sin(DH(2)) 0 0;
                sin(DH(2))  cos(DH(2)) 0 0;
                   0            0      1 0;
                   0            0      0 1];
        Transa = [1 0 0 DH(3);
                  0 1 0 0;
                  0 0 1 0;
                  0 0 0 1];
        Rotx = [   1  0  0  0;
                  0  cos(DH(4)) -sin(DH(4)) 0;
                  0  sin(DH(4))  cos(DH(4)) 0;                  
                  0      0            0     1];    
        A_i= Transd*Rotz*Transa*Rotx;
        Mul = Mul*A_i;
end

fprintf('The final Transformation Matrix is: \n');
Mul

fprintf('The final position of End effector (x,y,z) in inertial frame:\n');
Mul(1:3,4)