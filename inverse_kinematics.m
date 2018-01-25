%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inverse Kinematics 
% Mayank Roy
% IIT Delhi
% This program calulates the joint angle for a given end effector pose
% and position for a 2DOF Manipulator
%%%%%%%%%%%%%%%%%%%%%%%%

a1 = 1;a2 = 1; a3 =0;
%given circle center  = (0,0) , radius = 4,psi = pi/4;
%theta = 0:0.1:2*pi;
r = 1;
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

Q = [R T;zeros(1,3),1]
         
fx = 0.5;
fy = 0.5;
tu = 0;
tv = 0;

CI = [fx 0 tu; 0 fy tv; 0 0 1];


figure(1);
for    theta = 0:0.1:8*pi
    
    t =  0:0.1:3*pi;
    qx = r*cos(t);
    qy = r*sin(t);
    plot(qx,qy);
    hold on; 
    
    Wx = r*cos(theta);
    Wy = r*sin(theta);


    plot(p(1),p(2),'y.','MarkerSize',30);
    
    %(Wx^2 + Wy^2 - a1^2 - a2^2)/(2*a1*a2);
    theta2 = acos((Wx^2 + Wy^2 - a1^2 - a2^2)/(2*a1*a2));

    A1 = a1+a2*cos(theta2);
    A2 = a2*sin(theta2);

    stheta1 = (Wy*(a1 +a2*cos(theta2)) - Wx * a2 * sin(theta2))/(a1^2 + a2^2 + 2*a1*a2*cos(theta2));
    ctheta1 = (Wx*(a1 +a2*cos(theta2)) + Wy * a2 * sin(theta2))/(a1^2 + a2^2 + 2*a1*a2*cos(theta2));
    theta1 = atan2(stheta1,ctheta1);    
    %theta3 = psi - (theta1 + theta2);
    
    link = [a1*cos(theta1),a1*sin(theta1);
           Wx,Wy;];
    
    
    plot([0 link(1,1)],[0 link(1,2)],'b-','LineWidth',2);
    plot(link(1,1),link(1,2),'b.','MarkerSize',30);
  
    plot([link(1,1) link(2,1)],[link(1,2) link(2,2)],'b-','LineWidth',2);
    plot( link(2,1),link(2,2),'r.','MarkerSize',30);
 
    axis([-4 4 -4 4]);
    plot(Wx,Wy);
    pause(0.08);
    hold off;
    
    
end



