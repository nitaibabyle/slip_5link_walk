
clear functions %
clear all

%%%%%%%%% INITIALIZE PARAMETERS %%%%%%
%Mechanical parameters.

slip.m1   =  1;
slip.m2  =  0.2;    
slip.m3  =  0.2;  % masses
slip.I1  =  3*eye(3);  
slip.I2  =  0.5*eye(3);  % inertias about cmsslip.l1   =  1
slip.I3  =  0.5*eye(3);
slip.a1  =  0.5;            % dist. from O to G1 and E to G2 (see figures)
slip.a2  =  0.5;
slip.a3  =  0.5;
slip.l1  =  1;
slip.l2  =  1;
slip.l3  =  1;
slip.g   =  10;
slip.leg =  1.732;
slip.theta= pi/8;
slip.kp  =  100;
slip.kd  =  0;
slip.control_kp=0.12;
fps=50;

% Initial conditions and other settings.
framespersec=1000;  %if view is not speeded or slowed in dbpend_animate
T=5;             %duration of animation  in seconds

x=0;
xd=0;
y=3;
yd=0;
phi=0; 
dphi=0;
q1    = pi/5; %angle made by link1 with vertical
qd1    = 0;        %abslolute velocity of link1   
q2    = pi*2/6 ;      %angle made by link2 with vertical
qd2    = 0;        %abslolute velocity of link2
slip.final=q1-q2+pi/2;
z0=[x xd y yd phi dphi phi-q1-q2-pi*3/2 qd1 q2 qd2];


t0=0;
dt=2;
t_ode=t0;
[P1,P2,P3]=kinematic(z0(5:10),0,slip);
z_ode=[z0 x-P2(1) y-P2(2) x-P3(1) y-P3(2)];
fps = 50;
N=1000;


for j=1:5
%%%%%%% INTEGRATOR or ODE SOLVER %%%%%%%

% theta=slip.control_kp*(z0(2)-0.05);
% z0(5)=theta+pi/2-z0(7)/2+pi/38;
options = odeset('Abstol',1e-13,'Reltol',1e-13,'Events',@contact);
tspan = linspace(t0,t0+dt,dt*N);
[t_1 z_1] = ode113(@fly,tspan,z0,options,slip);
% z_1(:,5)=linspace(slip.final,z0(5),length(t_1));
for i=1:length(z_1)
    [P1,P2,P3]=kinematic(z_1(i,7:10),i,slip);
    P(i,1)=P2(1);
    P(i,2)=P2(2);
    P(i,3)=P3(1);
    P(i,4)=P3(2);  
end

z_1=[z_1,z_1(:,1)-P(:,1),z_1(:,3)-P(:,2),z_1(:,1)-P(:,3),z_1(:,3)-P(:,4)];
[t_1,z_1] = loco_interpolate(t_1,z_1,fps);
P=0;
t0 = t_1(end);
z0(1:10)=z_1(end,1:10);
% z0(5)=z0(5)-z0(7)+pi/2;
% z0(7)=z0(7);
J=[-(slip.m2*(slip.a2*(cos(z_1(7))*sin(z_1(9)) + cos(z_1(9))*sin(z_1(7))) + slip.l1*sin(z_1(7))) + slip.m1*(slip.l2*(cos(z_1(7))*sin(z_1(9)) + cos(z_1(9))*sin(z_1(7))) + slip.l1*sin(z_1(7)) + a3*(cos(q3)*(cos(z_1(7))*sin(z_1(9)) + cos(z_1(9))*sin(z_1(7))) + sin(q3)*(cos(z_1(7))*cos(z_1(9)) - sin(z_1(7))*sin(z_1(9))))) + slip.a1*slip.m3*sin(z_1(7)))/(slip.m3 + slip.m2 + slip.m1), -(slip.m1*(slip.l2*(cos(z_1(7))*sin(z_1(9)) + cos(z_1(9))*sin(z_1(7))) + a3*(cos(q3)*(cos(z_1(7))*sin(z_1(9)) + cos(z_1(9))*sin(z_1(7))) + sin(q3)*(cos(z_1(7))*cos(z_1(9)) - sin(z_1(7))*sin(z_1(9))))) + slip.a2*slip.m2*(cos(z_1(7))*sin(z_1(9)) + cos(z_1(9))*sin(z_1(7))))/(slip.m3 + slip.m2 + slip.m1), -(a3*slip.m1*(cos(q3)*(cos(z_1(7))*sin(z_1(9)) + cos(z_1(9))*sin(z_1(7))) + sin(q3)*(cos(z_1(7))*cos(z_1(9)) - sin(z_1(7))*sin(z_1(9)))))/(slip.m3 + slip.m2 + slip.m1);
     (slip.m2*(slip.a2*(cos(z_1(7))*cos(z_1(9)) - sin(z_1(7))*sin(z_1(9))) + slip.l1*cos(z_1(7))) + slip.m1*(slip.l2*(cos(z_1(7))*cos(z_1(9)) - sin(z_1(7))*sin(z_1(9))) + slip.l1*cos(z_1(7)) + a3*(cos(q3)*(cos(z_1(7))*cos(z_1(9)) - sin(z_1(7))*sin(z_1(9))) - sin(q3)*(cos(z_1(7))*sin(z_1(9)) + cos(z_1(9))*sin(z_1(7))))) + slip.a1*slip.m3*cos(z_1(7)))/(slip.m3 + slip.m2 + slip.m1),  (slip.m1*(slip.l2*(cos(z_1(7))*cos(z_1(9)) - sin(z_1(7))*sin(z_1(9))) + a3*(cos(q3)*(cos(z_1(7))*cos(z_1(9)) - sin(z_1(7))*sin(z_1(9))) - sin(q3)*(cos(z_1(7))*sin(z_1(9)) + cos(z_1(9))*sin(z_1(7))))) + slip.a2*slip.m2*(cos(z_1(7))*cos(z_1(9)) - sin(z_1(7))*sin(z_1(9))))/(slip.m3 + slip.m2 + slip.m1),  (a3*slip.m1*(cos(q3)*(cos(z_1(7))*cos(z_1(9)) - sin(z_1(7))*sin(z_1(9))) - sin(q3)*(cos(z_1(7))*sin(z_1(9)) + cos(z_1(9))*sin(z_1(7)))))/(slip.m3 + slip.m2 + slip.m1)];


qd=J\[z0(2);z0(4)];
z0(8)=qd(1);
z0(10)=qd(2);


options1 = odeset('Abstol',1e-13,'Reltol',1e-13,'Events',@release);
tspan = linspace(t0,t0+dt,dt*N);
[t_2 z_2] = ode113(@rhs_3link,tspan,z0,options1,slip);
  
[t_2,z_2] = loco_interpolate(t_2,z_2,fps);
z_2=[z_2,z_1(end,11)*ones(length(t_2),1),zeros(length(t_2),1)];
t0 = t_2(end);
z0(1:10)=z_2(end,1:10);
% z_2(:,5)=z_2(:,5)+z_2(:,7)-pi/2;
% z_2(:,7)=z_2(:,7);
t_ode=[t_ode;t_1(2:end);t_2(2:end)];
z_ode=[z_ode;z_1(2:end,:);z_2(2:end,:)];


[P11,P22,P33]=kinematic(z0(7:10),i,slip);
J=[-(slip.m2*(slip.a2*(cos(z_1(7))*sin(z_1(9)) + cos(z_1(9))*sin(z_1(7))) + slip.l1*sin(z_1(7))) + slip.m1*(slip.l2*(cos(z_1(7))*sin(z_1(9)) + cos(z_1(9))*sin(z_1(7))) + slip.l1*sin(z_1(7)) + a3*(cos(q3)*(cos(z_1(7))*sin(z_1(9)) + cos(z_1(9))*sin(z_1(7))) + sin(q3)*(cos(z_1(7))*cos(z_1(9)) - sin(z_1(7))*sin(z_1(9))))) + slip.a1*slip.m3*sin(z_1(7)))/(slip.m3 + slip.m2 + slip.m1), -(slip.m1*(slip.l2*(cos(z_1(7))*sin(z_1(9)) + cos(z_1(9))*sin(z_1(7))) + a3*(cos(q3)*(cos(z_1(7))*sin(z_1(9)) + cos(z_1(9))*sin(z_1(7))) + sin(q3)*(cos(z_1(7))*cos(z_1(9)) - sin(z_1(7))*sin(z_1(9))))) + slip.a2*slip.m2*(cos(z_1(7))*sin(z_1(9)) + cos(z_1(9))*sin(z_1(7))))/(slip.m3 + slip.m2 + slip.m1), -(a3*slip.m1*(cos(q3)*(cos(z_1(7))*sin(z_1(9)) + cos(z_1(9))*sin(z_1(7))) + sin(q3)*(cos(z_1(7))*cos(z_1(9)) - sin(z_1(7))*sin(z_1(9)))))/(slip.m3 + slip.m2 + slip.m1);
     (slip.m2*(slip.a2*(cos(z_1(7))*cos(z_1(9)) - sin(z_1(7))*sin(z_1(9))) + slip.l1*cos(z_1(7))) + slip.m1*(slip.l2*(cos(z_1(7))*cos(z_1(9)) - sin(z_1(7))*sin(z_1(9))) + slip.l1*cos(z_1(7)) + a3*(cos(q3)*(cos(z_1(7))*cos(z_1(9)) - sin(z_1(7))*sin(z_1(9))) - sin(q3)*(cos(z_1(7))*sin(z_1(9)) + cos(z_1(9))*sin(z_1(7))))) + slip.a1*slip.m3*cos(z_1(7)))/(slip.m3 + slip.m2 + slip.m1),  (slip.m1*(slip.l2*(cos(z_1(7))*cos(z_1(9)) - sin(z_1(7))*sin(z_1(9))) + a3*(cos(q3)*(cos(z_1(7))*cos(z_1(9)) - sin(z_1(7))*sin(z_1(9))) - sin(q3)*(cos(z_1(7))*sin(z_1(9)) + cos(z_1(9))*sin(z_1(7))))) + slip.a2*slip.m2*(cos(z_1(7))*cos(z_1(9)) - sin(z_1(7))*sin(z_1(9))))/(slip.m3 + slip.m2 + slip.m1),  (a3*slip.m1*(cos(q3)*(cos(z_1(7))*cos(z_1(9)) - sin(z_1(7))*sin(z_1(9))) - sin(q3)*(cos(z_1(7))*sin(z_1(9)) + cos(z_1(9))*sin(z_1(7)))))/(slip.m3 + slip.m2 + slip.m1)];
V=J*[z0(8);z0(10)];
z0(1)=z_2(end,11)+P33(1);
z0(3)=z_2(end,12)+P33(2);
z0(2)=V(1);
z0(4)=V(2);
slip.final=z0(5);
end

%%%%%%% POSTPROCESSING %%%%%%%%%%%%%%%%
% A routine to animate the results
% To speed up change the framespersecond


t=t_ode;
z=z_ode;

figure(1)

%     q1=z(i,1);
%     q2=z(i,3);
%     
%     xm1=slip.l1*sin(q1);%-l*sin(z(i,1));%modified by cxc in Jan. 2018
%     ym1=slip.l1*-cos(q1);
%     zm1=0;
%     xm2=slip.l1*sin(q1) + slip.l2*sin(q1-q2);%xm1-l*sin(z(i,3));%modified by cxc in Jan. 2018
%     ym2=slip.l1*-cos(q1) + slip.l2*-cos(q1-q2);
%     zm2=0;
% for i=1:length(t)
%     [P1,P2]=kinematic1(z(i,5:8),i,slip);
%     window_xmin = -3*slip.l1; window_xmax = 3*slip.l1;
%     window_ymin = -3*slip.l1; window_ymax = 3*slip.l1;
%     plot(z(i,1),z(i,3),'ko','MarkerSize',3); %pivot point
%     line([z(i,1) z(i,1)+P1(1)],[z(i,3) z(i,3)+P1(2)],'Linewidth',4,'Color',[0.8 0 0]);% first pendulum
%     line([z(i,1)+P1(1) z(i,1)+P2(1)],[z(i,3)+P1(2) z(i,3)+P2(2)],'Linewidth',4,'Color',[0 0.8 0]);% second pendulum
%     axis('equal')
%     axis on
%     axis([window_xmin window_xmax window_ymin window_ymax])
%     F(i)=getframe;
% end
for i=1:length(t)
    [P11,P22]=kinematic(z(i,5:8),i,slip);
    window_xmin = -2*slip.l1; window_xmax = 3*slip.l1;
    window_ymin = -0.5*slip.l1; window_ymax = 3.5*slip.l1;
    ttttt=[P22(1)+z(i,9),P22(2)+z(i,10)];
    plot(P22(1)+z(i,9),P22(2)+z(i,10),'ko','MarkerSize',3); %pivot point
    line([P22(1)+z(i,9) P11(1)+z(i,9)],[P22(2)+z(i,10) P11(2)+z(i,10)],'Linewidth',4,'Color',[0.8 0 0]);% first pendulum
    line([P11(1)+z(i,9) z(i,9)],[P11(2)+z(i,10) z(i,10)],'Linewidth',4,'Color',[0 0.8 0]);% second pendulum
    axis('equal')
    axis on
    axis([window_xmin window_xmax window_ymin window_ymax])
    F(i)=getframe;
end

% for i=1:length(t_3)
%     [P1,P2]=kinematic1(z_3(i,5:8),i,slip);
%     window_xmin = -3*slip.l1; window_xmax = 3*slip.l1;
%     window_ymin = -3*slip.l1; window_ymax = 3*slip.l1;
%     plot(z_3(i,1),z_3(i,3),'ko','MarkerSize',3); %pivot point
%     line([z_3(i,1) z_3(i,1)+P1(1)],[z_3(i,3) z_3(i,3)+P1(2)],'Linewidth',4,'Color',[0.8 0 0]);% first pendulum
%     line([z_3(i,1)+P1(1) z_3(i,1)+P2(1)],[z_3(i,3)+P1(2) z_3(i,3)+P2(2)],'Linewidth',4,'Color',[0 0.8 0]);% second pendulum
%     axis('equal')
%     axis on
%     axis([window_xmin window_xmax window_ymin window_ymax])
%     F(i)=getframe;
% end
v = VideoWriter('slipmodel6.avi');
open(v);
writeVideo(v,F);
close(v);

% for i=1:length(t)
%     [KE(i), PE(i)] = energy(t(i),z(i,:),slip);
% end
% TE = KE + PE;
% TE_diff = diff(TE);
% 
% figure(3)
% plot(t(1:end-1),TE_diff)
