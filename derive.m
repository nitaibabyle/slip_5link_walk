close all; clc;clear;

syms x z phi leg_hip_angle_left leg_knee_angle_left leg_hip_angle_right leg_knee_angle_right 'real' % joint angle, angular rate and acceleration
syms dx dz dphi dleg_hip_angle_left dleg_knee_angle_left dleg_hip_angle_right dleg_knee_angle_right 'real'             % CoM distance relative to parent joint and link length
syms ddx ddz ddphi ddleg_hip_angle_left ddleg_knee_angle_left ddleg_hip_angle_right ddleg_knee_angle_right 'real'
syms qdd 'real'
syms M m_left_thigh m_left_shank m_right_thigh m_right_shank M 'real'     % mass of links and motors
syms I1_1  I1_12 I1_13 I1_12 I1_2  I1_23 I1_13 I1_23 I1_3 I2_1 I2_12 I2_13 I2_12 I2_2  I2_23 I2_13 I2_23 I2_3 Itrunk1 Itrunk2 Itrunk3 'real'     % inertia moment of links and motors
syms g 'real'                       % gravatical acceleration
syms thigh_length shank_length trunk_length 'real'
syms thigh_center_pos shank_center_pos trunk_center_pos 'real' 
syms tal_hip_left tal_hip_right tal_knee_left tal_knee_right 'real'   
syms F1x F1z F2x F2z 'real'   

%%%注意两杆的质心位置，即transformation
%%%matrix还有动能势能的表达写法，
%%%最重要的是在自己定制的坐标系下，角度与角速度要统一，
%%%就是自己定制坐标系肯定存在一个特定的旋转矩阵，后面相应的角速度也应该跟随这一矩阵而调整
%%%主函数中，验证正确性时，用到的能量图像，也应该跟随这样的旋转矩阵而产生相应的调整，
%%%总结一下就是，先检查旋转矩阵、角速度等，derive后不仅更新mm与ff还要更新ke、kp，最终才能完整验证。
%%%20211111 zjt
I1=[I1_1  I1_12 I1_13;
    I1_12 I1_2  I1_23;
    I1_13 I1_23 I1_3];%%%大腿惯量
I2=[I2_1  I2_12 I2_13;
    I2_12 I2_2  I2_23;
    I2_13 I2_23 I2_3];%%%小腿惯量
Itrunk=[Itrunk1        0        0;
              0  Itrunk2        0;
              0        0  Itrunk3];
% unit vectors
i = [1; 0; 0];  j = [0; 1; 0];  k = [0; 0; 1];
% variables vectorization
q = [x;z;phi;leg_hip_angle_left;leg_knee_angle_left;leg_hip_angle_right;leg_knee_angle_right];
qd = [dx;dz;dphi;dleg_hip_angle_left;dleg_knee_angle_left;dleg_hip_angle_right;dleg_knee_angle_right];
qdd = [ddx;ddz;ddphi;ddleg_hip_angle_left;ddleg_knee_angle_left;ddleg_hip_angle_right;ddleg_knee_angle_right];
%qdd = [ddx;ddy;ddleg_theta_left;ddleg_length_left;ddleg_theta_right;ddleg_length_right];
G = -g * k; % gravity vector

%%%%运功学
Rtrunk=[cos(phi),  0, sin(phi);
               0,  1,         0;
        -sin(phi),  0,  cos(phi)]; %%%基座旋转矩阵

Rleft_thigh2torso=[cos(-leg_hip_angle_left),  0, sin(-leg_hip_angle_left);
                                         0,  1,                        0;
                   -sin(-leg_hip_angle_left),  0,  cos(-leg_hip_angle_left)]; %%%左大腿rotation matrix


Rleft_shank2thigh=[cos(leg_knee_angle_left),  0, sin(leg_knee_angle_left);
                                          0,  1,                         0;
                   -sin(leg_knee_angle_left),  0,  cos(leg_knee_angle_left)]; %%%左小腿rotation matrix    


Rright_thigh2torso=[cos(-leg_hip_angle_right),  0, sin(-leg_hip_angle_right);
                                           0,  1,                         0;
                    -sin(-leg_hip_angle_right),  0,  cos(-leg_hip_angle_right)]; %%%右大腿rotation matrix


Rright_shank2thigh=[cos(leg_knee_angle_right),  0, sin(leg_knee_angle_right);
                                            0,  1,                          0;
                    -sin(leg_knee_angle_right),  0,  cos(leg_knee_angle_right)]; %%%右小腿rotation matrix   
    

qicijuzhen=[0,0,0,1];

O=[x;0;z];%%%基座的原点位置
Ttrunk=[Rtrunk,     O;
           qicijuzhen];%%%上身相对世界坐标系的Trans矩阵
P_end_trunk=Ttrunk*[0;0;trunk_length;1];%%%上身顶点的位置
P_center_trunk=Ttrunk*[0;0;trunk_center_pos;1];%%%上身重心位置

O1=[0;0;0];%%%大腿相对于上身的Trans矩阵
T_left_thigh = [Rleft_thigh2torso,     O1;
                             qicijuzhen];%%%
T_right_thigh = [Rright_thigh2torso,     O1;
                               qicijuzhen];%%%
P_end_thigh_left=Ttrunk*T_left_thigh*[0;0;-thigh_length;1];%%%大腿末端位置
P_end_thigh_right=Ttrunk*T_right_thigh*[0;0;-thigh_length;1];  
P_center_thigh_left=Ttrunk*T_left_thigh*[0;0;-thigh_center_pos;1];%%%大腿重心位置
P_center_thigh_right=Ttrunk*T_right_thigh*[0;0;-thigh_center_pos;1];

O2=[0;0;-thigh_length];%%%小腿相对于大腿的Trans矩阵                     
T_left_shank = [Rleft_shank2thigh,     O2;
                             qicijuzhen];
T_right_shank = [Rright_shank2thigh,     O2;
                               qicijuzhen];                         
P_end_shank_left = Ttrunk*T_left_thigh*T_left_shank*[0;0;-shank_length;1];%%%足端的位置
P_end_shank_right = Ttrunk*T_right_thigh*T_right_shank*[0;0;-shank_length;1]; 
P_center_shank_left = Ttrunk*T_left_thigh*T_left_shank*[0;0;-shank_center_pos;1];%%%小腿的重心位置
P_center_shank_right = Ttrunk*T_right_thigh*T_right_shank*[0;0;-shank_center_pos;1]; 

%%%%雅可比
J_left_thigh=jacobian(P_center_thigh_left(1:3),q);
J_right_thigh=jacobian(P_center_thigh_right(1:3),q);
J_left_shank=jacobian(P_center_shank_left(1:3),q);
J_right_shank=jacobian(P_center_shank_right(1:3),q);
J_torso=jacobian(P_center_trunk(1:3),q);

% J_left_foot=jacobian(P_end_shank_left(1:3),q)
% J_right_foot=jacobian(P_end_shank_right(1:3),q)

J_left_foot=jacobian(P_end_shank_left(1:3)-[x;0;z],q);
J_right_foot=jacobian(P_end_shank_right(1:3)-[x;0;z],q);

% J_left_foot=jacobian(P_end_shank_left(1:3)-[x;0;z],q(4:5));
% J_right_foot=jacobian(P_end_shank_right(1:3)-[x;0;z],q(6:7));

Vthigh_left = jacobian(P_center_thigh_left(1:3),q)*qd;         % 左大腿重心速度
Vshank_left = jacobian(P_center_shank_left(1:3),q)*qd;         % 左小腿重心速度
Vthigh_right = jacobian(P_center_thigh_right(1:3),q)*qd;       % 右大腿重心速度
Vshank_right = jacobian(P_center_shank_right(1:3),q)*qd;       % 右小腿重心速度
Vtorso = jacobian(P_center_trunk(1:3),q)*qd;                   % 上身重心速度

Vfoot_left = jacobian(P_end_shank_left(1:3),q)*qd;
Vfoot_right = jacobian(P_end_shank_right(1:3),q)*qd;
% angular velocity

W_trunk = dphi*j;
W_left_thigh=(dphi-dleg_hip_angle_left)*j;
W_left_shank=(dphi-dleg_hip_angle_left+dleg_knee_angle_left)*j;
W_right_thigh=(dphi-dleg_hip_angle_right)*j;
W_right_shank=(dphi-dleg_hip_angle_right+dleg_knee_angle_right)*j;


% W_m1 = k_r1 * dq1 * k;         % motor 1 angular velocity
% W_m2 = (dq1 + k_r2 * dq2) * k; % motor 2 angular velocity

% inertia momentum w.r.t. inertial frame(a.k.a. world frame)
% diag_inertia = @(input)( diag([0, 0, input]) ); % convert into diagonal inertia matrix

R1=Rtrunk*Rleft_thigh2torso;
R2=Rtrunk*Rleft_thigh2torso*Rleft_shank2thigh;
R3=Rtrunk*Rright_thigh2torso;
R4=Rtrunk*Rright_thigh2torso*Rright_shank2thigh;

I_left_thigh_w = R1 * I1 * R1';       % link   1 inertia
I_left_shank_w = R2 * I2 * R2';       % link   2 inertia
I_right_thigh_w = R3 * I1 * R3';       % link   1 inertia
I_right_shank_w = R4 * I2 * R4';       % link   2 inertia
I_trunk_w = Rtrunk * Itrunk * Rtrunk';       % link   2 inertia
% I_m1_w = R_m1 * diag_inertia(I_m1) * R_m1';   % motor  1 inertia
% I_m2_w = R_m2 * diag_inertia(I_m2) * R_m2';   % motor  2 inertia

B=[zeros(3,4);
    eye(4)];
u=[tal_hip_left,tal_hip_right,tal_knee_left,tal_knee_right]';

F_left=[F1x,0,F1z]';
F_right=[F2x,0,F2z]';

% kinematic energy
KE =0.5 * M * dot(Vtorso(1:3),Vtorso(1:3)) + ...
    0.5 * m_left_thigh * dot(Vthigh_left(1:3),Vthigh_left(1:3)) +...
    0.5 * m_right_thigh * dot(Vthigh_right(1:3),Vthigh_right(1:3))+...
    0.5 * m_left_shank * dot(Vshank_left(1:3),Vshank_left(1:3)) + ...
    0.5 * m_right_shank * dot(Vshank_right(1:3),Vshank_right(1:3))+...
    0.5 * W_trunk' * I_trunk_w * W_trunk +...
    0.5 * W_left_thigh' * I_left_thigh_w * W_left_thigh +...
    0.5 * W_right_thigh' * I_right_thigh_w * W_right_thigh +...
    0.5 * W_left_shank' * I_left_shank_w * W_left_shank +...
    0.5 * W_right_shank' * I_right_shank_w * W_right_shank;

% potential energy

PE = M * g * P_center_trunk(3) +...
    m_left_thigh * g * P_center_thigh_left(3)+...
    m_right_thigh * g * P_center_thigh_right(3)+...
    m_left_shank * g * P_center_shank_left(3)+...
    m_right_shank * g * P_center_shank_right(3);

%% Lagrangian derivation
L = simplify(KE - PE);
DL_Ddq = jacobian(L,qd);
% dDL_Ddq_dt - DL_Dq' = Tau
DL_Dq = jacobian(L,q);

dDL_Ddq_dt = jacobian(DL_Ddq, q) * qd + jacobian(DL_Ddq, qd) *qdd;


eqn = simplify(dDL_Ddq_dt - DL_Dq'- B * u - J_left_foot'* F_left - J_right_foot'* F_right);

[MM, FF] = equationsToMatrix(eqn, qdd); % convert equation to : MM * ddq = FF
MM
FF
