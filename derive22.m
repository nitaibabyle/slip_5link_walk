close all; clc;clear;

syms q1 q2 q3 qd1 qd2 qd3 qdd1 qdd2 qdd3 'real' % joint angle, angular rate and acceleration
syms l1 l2 l3 a1 a2 a3 'real'             % CoM distance relative to parent joint and link length
syms m1 m2 m3 'real'     % mass of links and motors
syms I1_1  I1_12 I1_13 I1_12 I1_2  I1_23 I1_13 I1_23 I1_3 I2_1 I2_12 I2_13 I2_12 I2_2  I2_23 I2_13 I2_23 I2_3 I3_1 I3_12 I3_13 I3_12 I3_2  I3_23 I3_13 I3_23 I3_3 'real'     % inertia moment of links and motors
syms g 'real'                       % gravatical acceleration
syms tal1 tal2 tal3 'real'   

%%%注意两杆的质心位置，即transformation
%%%matrix还有动能势能的表达写法，
%%%最重要的是在自己定制的坐标系下，角度与角速度要统一，
%%%就是自己定制坐标系肯定存在一个特定的旋转矩阵，后面相应的角速度也应该跟随这一矩阵而调整
%%%主函数中，验证正确性时，用到的能量图像，也应该跟随这样的旋转矩阵而产生相应的调整，
%%%总结一下就是，先检查旋转矩阵、角速度等，derive后不仅更新mm与ff还要更新ke、kp，最终才能完整验证。
%%%20211111 zjt
I1=[I1_1  I1_12 I1_13;
    I1_12 I1_2  I1_23;
    I1_13 I1_23 I1_3];
I2=[I2_1  I2_12 I2_13;
    I2_12 I2_2  I2_23;
    I2_13 I2_23 I2_3];
I3=[I3_1  I3_12 I3_13;
    I3_12 I3_2  I3_23;
    I3_13 I3_23 I3_3];


% unit vectors

i = [1; 0; 0];  j = [0; 1; 0];  k = [0; 0; 1];
% variables vectorization
q = [q1; q2; q3];
qd = [qd1; qd2; qd3];
qdd = [qdd1; qdd2; qdd3];
G = -g * j; % gravity vector

R1=[cos(q1), -sin(q1), 0;
    sin(q1),  cos(q1), 0;
          0,       0, 1];
O1=[0;0;0];
aaaa=[0,0,0,1];

R2=[cos(q2), -sin(q2), 0;
    sin(q2),  cos(q2), 0;
           0,       0, 1];  
R12=R1*R2;
O2=[l1;0;0];

R3=[cos(q3), -sin(q3), 0;
    sin(q3),  cos(q3), 0;
           0,       0, 1];  
R13=R1*R2*R3;
O3=[l2;0;0];

R_com_1=a1;
R_com_2=a2;
R_com_3=a3;

T1=[R1,O1;
    aaaa];
T2=[R2,O2;
    aaaa];
T3=[R3,O3;
    aaaa];
T12=T1*T2;
T13=T12*T3;
P1=T1*[l1;0;0;1];
P2=T12*[l2;0;0;1];
P3=T13*[l3;0;0;1];
mmmmmm=simplify(norm(P1-P2));
nnnnn=simplify(norm(P2));
xxxxx=simplify(P2(1:2)');
% ===== Position & Orientation =====
% P1 = l1 * sin(q1) * i + l1 * -cos(q1) * j; % CoM position of link 1 
% P2 = (l1 * sin(q1) + l2 * sin(q1 - q2)) * i + (l1 * -cos(q1) + l2 * -cos(q1 - q2)) * j; % CoM position of link 2

% P_ee = (a1 * cos(q1) + a2 * cos(q1 + q2)) * i + (a1 * sin(q1) + a2 * sin(q1 + q2)) * j; % end-effector position
% O_l1 = q1 * k; % orientation of link 1
% O_l2 = (q1 + q2) * k; % orientation of link 2
% 
% P_m1 = 0 * i + 0 * j; % position of motor 1
% P_m2 = a1 * cos(q1) * i + a1 * sin(q1) * j; % position of motor 2
% O_m1 = k_r1 * q1 * k; % orientation of motor 1 (multiply the gear ration 1)
% O_m2 = (q1 + k_r2 * q2) * k; % orientation of motor 2 (multiply the gear ration 2)
PG1=T1*[a1;0;0;1]; 
PG2=T12*[a2;0;0;1];
PG3=T13*[a3;0;0;1];
% T = aaa'*kp*[leg-sqrt(P_m1(1)^2+P_m2(1)^2);leg-sqrt(P_m1(2)^2+P_m2(2)^2)]; %弹簧阻尼，slip模型，根据末端位置的差值，得到最后的力矩值，需要用到运动学
% T1 = T(1);
% T2 = T(2); 
% 
% R1 = Rot('z', q1+pi/2);               % link 1 rotation matrix
% R2 = Rot('z', pi-q2);          % link 2 rotation matrix
% % R_m1 = Rot('z', k_r1 * q1);      % motor 1 rotation matrix
% % R_m2 = Rot('z', q1 + k_r2 * q2); % motor 2 rotation matrix

%derivative = @(input)( jacobian(input, q) * dq ); % anonymous function
                        % function output = derivative(input, q, dq) 
% same function as -->  % output = jacobian(input, q) * dq; 
                        % end

% absolute velocity (Note: All vectors are relative to the inertial frame!)
% linear velocity
J=jacobian(PG3,q)

PG = (PG1*m1+PG2*m2+PG3*m3)/(m1+m2+m3);
J1=jacobian(PG,q)
V_l1 = jacobian(PG1,q)*qd;       % link  1 CoM velocity
V_l2 = jacobian(PG2,q)*qd;       % link  2 CoM velocity
V_l3 = jacobian(PG3,q)*qd; 
V_mmm= jacobian(P2,q)*qd;

% V_m1 = derivative(P_m1);       % motor 1 velocity(point mass)
% V_m2 = derivative(P_m2);       % motor 2 velocity(point mass)
% angular velocity
W_l1 = qd1 * k;                % link  1 angular velocity
W_l2 = (qd1 + qd2) * k;        % link  2 angular velocity
W_l3 = (qd1 + qd2 + qd3) * k;
% W_m1 = k_r1 * dq1 * k;         % motor 1 angular velocity
% W_m2 = (dq1 + k_r2 * dq2) * k; % motor 2 angular velocity

% inertia momentum w.r.t. inertial frame(a.k.a. world frame)
% diag_inertia = @(input)( diag([0, 0, input]) ); % convert into diagonal inertia matrix
I_l1_w = R1 * I1 * R1';       % link   1 inertia
I_l2_w = R12 * I2 * R12';       % link   2 inertia
I_l3_w = R13 * I3 * R13';

% I_m1_w = R_m1 * diag_inertia(I_m1) * R_m1';   % motor  1 inertia
% I_m2_w = R_m2 * diag_inertia(I_m2) * R_m2';   % motor  2 inertia

% kinematic energy
KE = 0.5 * m1 * dot(V_l1, V_l1) + 0.5 *m2 * dot(V_l2, V_l2) + 0.5 *m3 * dot(V_l3, V_l3) + 0.5 * W_l1' * I_l1_w * W_l1 + 0.5 * W_l2' * I_l2_w * W_l2 + 0.5 * W_l3' * I_l3_w * W_l3;

% potential energy
pg1=PG1(1:3);
pg2=PG2(1:3);
pg3=PG3(1:3);

PE = m1 * g * PG1(2) + m2 * g * PG2(2) + m3 * g * PG3(2);

%% Lagrangian derivation
L = simplify(KE - PE);
DL_Ddq = jacobian(L,qd);
% dDL_Ddq_dt - DL_Dq' = Tau
DL_Dq = jacobian(L,q);

dDL_Ddq_dt = jacobian(DL_Ddq, q) * qd + jacobian(DL_Ddq, qd) *qdd;

tal=[tal1;tal2;tal3];
eqn = simplify(dDL_Ddq_dt - DL_Dq'-tal);

[MM, FF] = equationsToMatrix(eqn, qdd); % convert equation to : MM * ddq = FF
MM
FF
