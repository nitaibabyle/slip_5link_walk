clear functions %
clear all
clc
%%%%%%%%% INITIALIZE PARAMETERS %%%%%%
%Mechanical parameters.
slip.m_thigh=0.25;
slip.m_shank=0.3;
slip.M   =  2;
slip.Itrunk=  0.0267*eye(3);  
slip.I1  =  0.0075*eye(3);  
slip.I2  =  0.016*eye(3);  % inertias about cmsslip.l1   =  1

slip.thigh_length=0.3;
slip.shank_length=0.4;
slip.trunk_length=0.2;
slip.thigh_center_pos=0.1;
slip.shank_center_pos=0.2;
slip.trunk_center_pos=0.1;
slip.g=9.8;
slip.Llength=0.65;
slip.Rlength=0.65;

slip.flag=3;
slip.velocity=0;
global footplace;
footplace=zeros(1);
global velocity_current;
velocity_current=0;
% global theta_traj;
% theta_traj=-pi/2;

global angleplacement;
angleplacement=0;
global time;
time=0;
global startpoint;
%startpoint=0.06235;%%%双脚的中位点
startpoint=0.04;%%%单脚的中位点
%startpoint=0.055;%%%单脚的中位点
global velocity_filter;
velocity_filter=0;
slip.setstep=5;
% Initial conditions and other settings.
framespersec=1000;  %if view is not speeded or slowed in dbpend_animate
T=5;             %duration of animation  in seconds
left_angle=startpoint;
right_angle=startpoint;
x=2;
z=0.66;
phi=0;
[leg_hip_angle_left,leg_knee_angle_left,leg_hip_angle_right,leg_knee_angle_right]=slip2robot(zeros(3),slip.Llength,left_angle,slip.Rlength,right_angle,slip);
% leg_hip_angle_left=acos(0.904);
% leg_knee_angle_left=acos(0.71875);
% leg_hip_angle_right=acos(0.904);
% leg_knee_angle_right=acos(0.71875);

dx=0.00;
dz=0;
dphi=0.00;
dleg_hip_angle_left=0;
dleg_knee_angle_left=0;
dleg_hip_angle_right=0;
dleg_knee_angle_right=0;


z0=[x z phi leg_hip_angle_left leg_knee_angle_left leg_hip_angle_right leg_knee_angle_right ...
    dx dz dphi dleg_hip_angle_left dleg_knee_angle_left dleg_hip_angle_right dleg_knee_angle_right];

 t0=0;
% dt=0.001;
global t_ode;
global z_ode;
global step;
t_ode=t0;
z_ode=z0;
step=0;
 fps = 40;
% N=10000;

%%%%%%% INTEGRATOR or ODE SOLVER %%%%%%%


[t_ode,z_ode]=floatingbaseode(t0,z0,slip);

[t,zz] = loco_interpolate(t_ode,z_ode,fps);
%%%%%%% POSTPROCESSING %%%%%%%%%%%%%%%%
% A routine to animate the results
% To speed up change the framespersecond
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
    [P0,P1,P2,P3,P4,P5]=kinematic(zz(i,:),i,slip);
%     window_xmin = -1*1; window_xmax = 1*1;
%     window_ymin = -1*1; window_ymax = 1*1;
%     window_zmin = -1*1; window_zmax = 1*1;
%     plot3(P0(1),P0(2),P0(3),'ko','MarkerSize',8); %pivot point
%     line([P0(1) P1(1)],[P0(2) P1(2)],[P0(3) P1(3)],'Linewidth',4,'Color',[0.8 0 0]);% first pendulum
%     line([P0(1) P2(1)],[P0(2) P2(2)],[P0(3) P2(3)],'Linewidth',4,'Color',[0 0.8 0]);% second pendulum
%     line([P0(1) P3(1)],[P0(2) P3(2)],[P0(3) P3(3)],'Linewidth',4,'Color',[0 0.8 0.4]);% second pendulum
%     line([P2(1) P4(1)],[P2(2) P4(2)],[P2(3) P4(3)],'Linewidth',4,'Color',[0.4 0 0]);% second pendulum
%     line([P3(1) P5(1)],[P3(2) P5(2)],[P3(3) P5(3)],'Linewidth',4,'Color',[0 0.2 0.8]);% second pendulum
%     axis('equal')
%     axis on
%     axis([window_xmin window_xmax window_ymin window_ymax window_zmin window_zmax])
      window_xmin = -1*1; window_xmax = 4*1;
    window_ymin = -0.5*1; window_ymax = 1*1;
    window_zmin = -0.5*1; window_zmax = 1*1;
    plot(P0(1),P0(3),'ko','MarkerSize',8); %pivot point
    line([P0(1) P1(1)],[P0(3) P1(3)],'Linewidth',4,'Color',[0.8 0 0]);% first pendulum
    line([P0(1) P2(1)],[P0(3) P2(3)],'Linewidth',4,'Color',[0 0.8 0]);% second pendulum
    line([P0(1) P3(1)],[P0(3) P3(3)],'Linewidth',4,'Color',[0 0.8 0.4]);% second pendulum
    line([P2(1) P4(1)],[P2(3) P4(3)],'Linewidth',4,'Color',[0.4 0 0]);% second pendulum
    line([P3(1) P5(1)],[P3(3) P5(3)],'Linewidth',4,'Color',[0 0.2 0.8]);% second pendulum
    line([-6 6],[0 0],'Linewidth',4,'Color',[0 0 0]);% second pendulum
    
    axis('equal')
    axis on
    axis([window_xmin window_xmax window_ymin window_ymax])
   
    
    F(i)=getframe;
end
v = VideoWriter('ccc.avi');
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
% 
% 
% figure(4)
% plot(t(1:end),Pos)




function [t_ode,z_ode]=floatingbaseode(t0,z0,slip)
global t_ode;
global z_ode;

d_ground=-80;
k_ground=-2500;
d_groundx=-80;
dt=0.001;
global forcetime Lforce Rforce;
forcetime=0;
Lforce=0;
Rforce=0;
N=10000;

for i=1:25000
    
    
    %[x,y,leg_theta_left,leg_length_left,leg_theta_right,leg_length_right,dx,dy,dleg_theta_left,dleg_length_left,dleg_theta_right,dleg_length_right]=z0(1:end);
x=z0(1);
z=z0(2);
phi=z0(3);
leg_hip_angle_left=z0(4);
leg_knee_angle_left=z0(5);
leg_hip_angle_right=z0(6);
leg_knee_angle_right=z0(7);
dx=z0(8);
dz=z0(9);
dphi=z0(10);
dleg_hip_angle_left=z0(11);
dleg_knee_angle_left=z0(12);
dleg_hip_angle_right=z0(13);
dleg_knee_angle_right=z0(14);

[P0,P1,P2,P3,P4,P5]=kinematic(z0,i,slip);
[Vfoot_left,Vfoot_right]=velocity_of_foot(z0,i,slip);
nnn=(P4(3)>=0)*2+(P5(3)>=0);
%     if -1==sign(dy + dleg_length_left*(cos(leg_theta_left)*sin(phi) + cos(phi)*sin(leg_theta_left)) + dleg_theta_left*leg_length_left*(cos(leg_theta_left)*cos(phi) - sin(leg_theta_left)*sin(phi)) + dphi*leg_length_left*(cos(leg_theta_left)*cos(phi) - sin(leg_theta_left)*sin(phi)))
%         d_groundy=d_ground;
%     else
%         d_groundy=0;
%     end
%     
%     nnn=(y + leg_length_left*(cos(leg_theta_left)*sin(phi) + cos(phi)*sin(leg_theta_left))>=0)*2+(y + leg_length_right*(cos(leg_theta_right)*sin(phi) + cos(phi)*sin(leg_theta_right))>=0);
%     
    switch nnn
        
        case 0
            F_left=[d_groundx*Vfoot_left(1);0;k_ground*P4(3)+d_ground*Vfoot_left(3)];
            F_right=[d_groundx*Vfoot_right(1);0;k_ground*P5(3)+d_ground*Vfoot_right(3)];

            %%%%地面不会给负向力
            if F_left(3)<0
               F_left(3)=0;
            end
            if F_right(3)<0
               F_right(3)=0;
            end
            a=slip.flag;
            slip.flag=0;
            aaaa=0
            if slip.flag~=a
                slip.currentstep=a;
            end
        case 1
            F_left=[d_groundx*Vfoot_left(1);0;k_ground*P4(3)+d_ground*Vfoot_left(3)];
            F_right=zeros(3,1);
            if F_left(3)<0
                F_left(3)=0;
            end

            a=slip.flag;
            slip.flag=1;
            aaaa=1
            if slip.flag~=a
                slip.currentstep=a;
            end
        case 2
            F_left=zeros(3,1);
            F_right=[d_groundx*Vfoot_right(1);0;k_ground*P5(3)+d_ground*Vfoot_right(3)];
            
            if F_right(3)<0
                F_right(3)=0;
            end
            
            a=slip.flag;
            slip.flag=2;
            aaaa=2
            if slip.flag~=a
                slip.currentstep=a;
            end
        case 3
            F_left=zeros(3,1);
            F_right=zeros(3,1);
            a=slip.flag;
            slip.flag=3;
            aaaa=3
            if slip.flag~=a
                slip.currentstep=a;
            end
    end

         [left_hip_torque,left_knee_torque,right_hip_torque,right_knee_torque] = controller(t0,z0,slip);
%         Lforce=[Lforce;slip.f1];
%         Rforce=[Rforce;slip.f2];

        tal=[left_hip_torque,left_knee_torque,right_hip_torque,right_knee_torque];
        options1 = odeset('Abstol',1e-13,'Reltol',1e-13);
        tspan = linspace(t0,t0+dt,dt*N);
        [t_temp z_temp] = ode113(@rhs,tspan,z0,options1,slip,tal,F_left,F_right);
        
        t0 = t_temp(end,:);
        z0 = z_temp(end,:);
        t_ode = [t_ode;t_temp(2:end)];
        z_ode = [z_ode;z_temp(2:end,:)];
        forcetime=[forcetime;t0];
        if i>18000
            slip.velocity=0;
        else
            if i>15050
                slip.velocity=0.2;
            else
                if i>10050
                    %     z0(8)=-0.1;
                    slip.velocity=0;
                else
                    if i>7050
                        slip.velocity=-0.1;
                    end
                end
            end
        end
        
%         
        if (z0(2)<0)||(z0(2) + slip.trunk_length * cos(z0(3))<0)
            break;
        end

end
end