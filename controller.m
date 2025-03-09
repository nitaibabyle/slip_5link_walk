function [left_hip_torque,left_knee_torque,right_hip_torque,right_knee_torque]=controller(t,z0,slip)   
global time;
global velocity_current;
global footplacement;
global angleplacement;
global startpoint;
global footplace;
global velocity_filter;

x = z0(1);
z= z0(2);
phi = z0(3);
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



trunk_length=slip.trunk_length;
thigh_length=slip.thigh_length;
shank_length=slip.shank_length;
trunk_center_pos=trunk_length/2;
I1=0.0267;
I2=0.0267;

%%%控制器参数

slip.kp=-200;
slip.pinlv=1;
slip.gaodu=0.4;

k_velocity=0.3;
kd_velocity=-80;

kpswing=800;
kdswing=10;
torque1=0;
torque2=0;
T=0.35;
torso_kp=-600;
torso_kd=-5;

%%%腿长伸缩
if t<0.8
    left_length_ref=0.66;
    right_length_ref=0.66;
    omega=pi/T;
else
    
 k2=slip.gaodu;
    omega=pi/T;
    leg1 = sin(omega*t);
    leg2 = -sin(omega*t);
    if leg1<=0
        left_length_ref=slip.Llength*(1+k2*leg1);
        right_length_ref=slip.Rlength;
    else
         left_length_ref=slip.Llength;
         right_length_ref=slip.Rlength*(1+k2*leg2);
    end
end

    %%%产生slip降维模型
    [left_length,left_angle,right_length,right_angle]=robot2slip(z0,slip);
    base_pos=[x;0;z];
    [base_frame,top_pos,left_knee_pos,right_knee_pos,left_foot_pos,right_foot_pos]=kinematic(z0,1,slip);
    leftfoot_pos_wrt_base=base_pos-left_foot_pos;
    rightfoot_pos_wrt_base=base_pos-right_foot_pos;
    leglength=[left_length,right_length];
    F_slip=([left_length_ref,right_length_ref]-leglength)*slip.kp;
    F1=F_slip(1)*leftfoot_pos_wrt_base/norm(leftfoot_pos_wrt_base);
    F2=F_slip(2)*rightfoot_pos_wrt_base/norm(rightfoot_pos_wrt_base);
    
    [J_left_foot,J_right_foot]=jacobian_of_foot(z0,1,slip);
    tal_left=J_left_foot'*F1;
    tal_right=J_right_foot'*F2;
    ddphi=torso_kp*phi+torso_kd*dphi;
%     tal_torso=[-(F1(3)*trunk_center_pos*sin(phi) - ddphi*(I2*cos(phi)^2 + I1*sin(phi)^2) - F1(1)*trunk_center_pos*cos(phi) - (dphi^2*(2*I1*cos(phi)*sin(phi) - 2*I2*cos(phi)*sin(phi)))/2),...
%                -(F2(3)*trunk_center_pos*sin(phi) - ddphi*(I2*cos(phi)^2 + I1*sin(phi)^2) - F2(1)*trunk_center_pos*cos(phi) - (dphi^2*(2*I1*cos(phi)*sin(phi) - 2*I2*cos(phi)*sin(phi)))/2)];
     tal_torso=[torso_kp*phi+torso_kd*dphi,torso_kp*phi+torso_kd*dphi];
%%状态机
switch slip.flag
     case 1
        time=time+0.001;
        tal_left_torso=tal_torso(1);
        [~,~,leg_hip_angle_right,~]=slip2robot(z0,left_length,left_angle,right_length,traj(startpoint,angleplacement,time,omega),slip);
        tal_right_torso=kpswing*(leg_hip_angle_right-z0(6))-kdswing*z0(13);%%%
        
    case 2
        time=time+0.001;
        tal_right_torso=tal_torso(2);
        [leg_hip_angle_left,~,~,~]=slip2robot(z0,left_length,traj(startpoint,angleplacement,time,omega),right_length,right_angle,slip);
        tal_left_torso=kpswing*(leg_hip_angle_left-z0(4))-kdswing*z0(11);%%%
        
    case 3
        tal_right_torso=0;%%%
        tal_left_torso=0;%%%
%         [leg_hip_angle_left,~,leg_hip_angle_right,~]=slip2robot(z0,left_length_ref,traj(0,0,time,omega),right_length_ref,traj(0,0,time,omega),slip);
%         tal_right_torso=kpswing*(leg_hip_angle_right-z0(6))-kdswing*z0(13);%%%
%         tal_left_torso=kpswing*(leg_hip_angle_left-z0(4))-kdswing*z0(11);%%%
        
    case 0
        if slip.currentstep==1
  %           footplacement=asin(0.3124*z(8)*cos(z(3)))-k_velocity*(0-z(8)*cos(z(3)));
            footplacement=-k_velocity*(slip.velocity-z0(8)) + kd_velocity*(velocity_current-z0(8));
            if footplacement>=0.3
                footplacement=0.3;
            end
            if footplacement<=-0.3
                footplacement=-0.3;
            end
%                        angleplacement=-pi/2+footplacement;
             angleplacement=atan((footplacement+z0(8)*T/2)/0.58);
            footplace=[footplace;footplacement];
%             
%             theta_traj=((z(4)+footplacement)/2)-((cos((omega)*tt))*(footplacement-z(4))/2);
            [~,left_angle111,~,~]=robot2slip(z0,slip);
            startpoint=left_angle111;
        end
        if slip.currentstep==2
   %         footplacement=asin(0.3124*z(8)*cos(z(3)))-k_velocity*(0-z(8)*cos(z(3)));
            footplacement=-k_velocity*(slip.velocity-z0(8)) + kd_velocity*(velocity_current-z0(8));
            if footplacement>=0.3
                footplacement=0.3;
            end
            if footplacement<=-0.3
                footplacement=-0.3;
            end
            %angleplacement=-pi/2+footplacement;
            angleplacement=atan((footplacement+z0(8)*T/2)/0.58);
            footplace=[footplace;footplacement];
            %             theta_traj=((z(6)+footplacement)/2)-((cos((omega)*tt))*(footplacement-z(6))/2);
            [~,~,~,right_angle111]=robot2slip(z0,slip);
            startpoint=right_angle111;
        end
        %         torsotorque=(-(F1(3)*trunk_center_pos*sin(phi) - ddphi*(I2*cos(phi)^2 + I1*sin(phi)^2) - F1(1)*trunk_center_pos*cos(phi) - (dphi^2*(2*I1*cos(phi)*sin(phi) - 2*I2*cos(phi)*sin(phi)))/2)...
        %                      -(F2(3)*trunk_center_pos*sin(phi) - ddphi*(I2*cos(phi)^2 + I1*sin(phi)^2) - F2(1)*trunk_center_pos*cos(phi) - (dphi^2*(2*I1*cos(phi)*sin(phi) - 2*I2*cos(phi)*sin(phi)))/2))/2;
        torsotorque=torso_kp*phi+torso_kd*dphi;

        if F1(3)<0
            F1(3)=0.001;
        end
        if F2(3)<0
            F2(3)=0.001;
        end
        tal_left_torso=(F1(3))*torsotorque/(F1(3)+F2(3));
        tal_right_torso=(F2(3))*torsotorque/(F1(3)+F2(3));
        
        
        time=0;
      
end
velocity_current=z0(8);
left_hip_torque=tal_left_torso;
right_hip_torque=tal_right_torso;

left_knee_torque=tal_left(2)+dleg_knee_angle_left*-0.4;
right_knee_torque=tal_right(2)+dleg_knee_angle_right*-0.4;

%f1=0;
% f2=0;

% 
% tal1=0;
% 
% tal2=0;
% % % 
%   f1=0;
%   f2=0;

