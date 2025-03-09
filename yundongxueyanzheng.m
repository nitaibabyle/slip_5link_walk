clear functions %
clear all

framespersec=50;  %if view is not speeded or slowed in dbpend_animate
T=5;             %duration of animation  in seconds
tspan=linspace(0,T,T*framespersec);

x=0;
z=0.7;
phi=0;
q1 = pi/6+0.5; %angle made by link1 with vertical
q2 = pi/2; %angle made by link2 with vertical
q3 = pi/6-0.2; %angle made by link1 with vertical 
q4 = pi/2; %angle made by link2 with vertical
dx=0;dy=0;
dphi=0;dq1=0;dq2=0;dq3=0;dq4=0;
z0=[x,z,phi,q1,q2,q3,q4,dx,dy,dphi,dq1,dq2,dq3,dq4]';
slip.trunk_length=0.2;
slip.thigh_length=0.3;
slip.shank_length=0.4;
[P0,P1,P2,P3,P4,P5]=kinematic(z0,i,slip);

figure(1)
plot3(P0(1),P0(2),P0(3),'ko','MarkerSize',8); %pivot point
hold on
line([P0(1) P1(1)],[P0(2) P1(2)],[P0(3) P1(3)],'Linewidth',4,'Color',[0.8 0 0]);% first pendulum
line([P0(1) P2(1)],[P0(2) P2(2)],[P0(3) P2(3)],'Linewidth',4,'Color',[0 0.8 0]);% second pendulum
line([P0(1) P3(1)],[P0(2) P3(2)],[P0(3) P3(3)],'Linewidth',4,'Color',[0 0.8 0.4]);% second pendulum
line([P2(1) P4(1)],[P2(2) P4(2)],[P2(3) P4(3)],'Linewidth',4,'Color',[0.4 0 0]);% second pendulum
line([P3(1) P5(1)],[P3(2) P5(2)],[P3(3) P5(3)],'Linewidth',4,'Color',[0 0.2 0.8]);% second pendulum
line([-5 5],[0 0],[0 0],'Linewidth',4,'Color',[0 0.2 0.8]);
axis([-2*0.5 2*0.5 -2*0.5 2*0.5 -2*0.5 2*0.5]);
axis square
hold off