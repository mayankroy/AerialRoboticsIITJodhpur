%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control
% Mayank Roy
% IIT Delhi
% This program illustrates PD Control, Feedforward Control and 
% Torque based Controllers.
%%%%%%%%%%%%%%%%%%%%%%%%%%

% Run it. This is main program.

%Initial conditions
th1=0;dth1=0;th2 = 0;dth2 =0;
y0=[th1; dth1;th2;dth2];
%Time period
T=3;
%Integrating
[T,Y] = ode45(@ode2link,[0 T],y0);

%Plotting
figure(1)
plot(T,Y)
set (gca,'fontsize',10,'fontweight','n','fontname','times new romans','linewidth',0.5,'Box', 'off','TickDir','out' );
xlabel('time (s)','FontSize',10);
ylabel('State variables','FontSize',10);
legend('Joint angle 1(rad)','Joint velocity 1(rad/s)','Joint angle 2(rad)','Joint velocity 2(rad/s)')

% Animating
le=length(T);
l=1;
for i=1:le
    %state variables
    th1=Y(i,1);
    th2=Y(i,3);
    %co-ordinates of fixed pivot
    x0=0;
    y0=0;
    %co-ordinates of end effector
    x1=l*cos(th1);
    y1=l*sin(th1);
    x2 = l*cos(th1)+l*cos(th1+th2);
    y2 = l*sin(th1)+l*sin(th1+th2);

    xx=[x0 x1 x2];
    yy=[y0 y1 y2];
    % Animating
    figure(2)
    plot(xx,yy)
    set (gca,'fontsize',10,'fontweight','n','fontname','times new romans','linewidth',0.5,'Box', 'off','TickDir','out' );
    axis([-2 2 -2 2])
    xlabel('X (m)','FontSize',10);
    ylabel('Y (m)','FontSize',10);
    pause(0.01)
    drawnow;
end
    
    
