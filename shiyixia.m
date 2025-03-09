clear functions %
clear all

%%%%%%%%% INITIALIZE PARAMETERS %%%%%%
%Mechanical parameters.

slip.m3   =  1;
slip.m1  =  0.2;    
slip.m2  =  0.2;  % masses
slip.I1  =  0.5*eye(3);  
slip.I2  =  0.5*eye(3);  % inertias about cmsslip.l1   =  1
slip.I3  =  2*eye(3);
slip.a1  =  0.5;            % dist. from O to G1 and E to G2 (see figures)
slip.a2  =  0.5;
slip.a3  =  0.5;
slip.l1  =  1;
slip.l2  =  1;
slip.l3  =  1;
slip.g   =  10;
slip.leg =  2.0128;
slip.kp  =  200;
slip.kd  =  2;


% Initial conditions and other settings.
framespersec=1000;  %if view is not speeded or slowed in dbpend_animate
T=5;             %duration of animation  in seconds

x=0;
xd=0;
y=0;
yd=0;
q1    = pi*80/180; %angle made by link1 with vertical
%qd1    =-4.8506;        %abslolute velocity of link1   
qd1    =0;
q2    = pi*30/180 ;      %angle made by link2 with vertical
%qd2    =6.4062;        %abslolute velocity of link2
qd2    =0;
q3      =-pi/9;
qd3       =  0;
q4      =-pi/9;
qd4       =  0;
q5      =-pi/9;
qd5       =  0;
z0=[x xd y yd q1 qd1 q2 qd2 q3 qd3];


t0=0;
dt=20;
t_ode=t0;
z_ode=z0;
fps = 30;
N=1000;

%%%%%%% INTEGRATOR or ODE SOLVER %%%%%%%


options1 = odeset('Abstol',1e-13,'Reltol',1e-13,'Events',@release);
tspan = linspace(t0,t0+dt,dt*N);
[t_2 z_2] = ode113(@rhs_3link,tspan,z0,options1,slip);
[t_2,z_2] = loco_interpolate(t_2,z_2,fps);
t=t_2;
z=z_2;

%%%%%%% POSTPROCESSING %%%%%%%%%%%%%%%%
% A routine to animate the results
% To speed up change the framespersecond
% for i=1:length(t)
%     [KE(i), PE(i)] = energy(t(i),z(i,:),slip);
% end
% TE = KE + PE;
% TE_diff = diff(TE);
% % figure(3)
% 
% plot(t(1:end-1),TE_diff)


figure(1)


for i=1:length(t)

%     q1=z(i,1);
%     q2=z(i,3);
%     
%     xm1=slip.l1*sin(q1);%-l*sin(z(i,1));%modified by cxc in Jan. 2018
%     ym1=slip.l1*-cos(q1);
%     zm1=0;
%     xm2=slip.l1*sin(q1) + slip.l2*sin(q1-q2);%xm1-l*sin(z(i,3));%modified by cxc in Jan. 2018
%     ym2=slip.l1*-cos(q1) + slip.l2*-cos(q1-q2);
%     zm2=0;

    [P1,P2,P3]=kinematic(z(i,5:10),i,slip);
Pos(i)=norm(P2);
    window_xmin = -3*slip.l1; window_xmax = 3*slip.l1;
    window_ymin = -3*slip.l1; window_ymax = 3*slip.l1;
    plot(z(i,1),z(i,3),'ko','MarkerSize',3); %pivot point
    line([z(i,1) z(i,1)+P1(1)],[z(i,3) z(i,3)+P1(2)],'Linewidth',4,'Color',[0.8 0 0]);% first pendulum
    line([z(i,1)+P1(1) z(i,1)+P2(1)],[z(i,3)+P1(2) z(i,3)+P2(2)],'Linewidth',4,'Color',[0 0.8 0]);% second pendulum
    line([z(i,1)+P2(1) z(i,1)+P3(1)],[z(i,3)+P2(2) z(i,3)+P3(2)],'Linewidth',4,'Color',[0 0 0.8]);
    axis('equal')
    axis on
    axis([window_xmin window_xmax window_ymin window_ymax])
    F(i)=getframe;
end
v = VideoWriter('ccc.avi');
open(v);
writeVideo(v,F);
close(v);





