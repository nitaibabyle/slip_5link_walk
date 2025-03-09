close all; clc;clear;

syms x z phi 'real' % joint angle, angular rate and acceleration
syms dx dz dphi 'real'             % CoM distance relative to parent joint and link length
syms ddx ddz ddphi 'real'
syms qdd 'real'
syms M trunk_center_pos 'real'     % mass of links and motors
syms I1 I2 I3 'real'     % inertia moment of links and motors
syms g 'real'                       % gravatical acceleration
syms F1x F1y F2x F2zz F3x F3z F4x F4z Fforcex Fforcez F1 F2 tal1 'real'

%%%20220227 zjt
I=[I1, 0;
    0,I2];
% unit vectors
i = [1; 0];  j = [0; 1];  k = [0; 0; 1];
% variables vectorization
q = [x;z;phi];
qd = [dx;dz;dphi];
qdd = [ddx;ddz;ddphi];
%qdd = [ddx;ddy;ddleg_theta_left;ddleg_length_left;ddleg_theta_right;ddleg_length_right];
G = -g * k; % gravity vector

%%%%运功学四个顶点，正方形如下
%%%     P1--------P2
%%%     |    ↑z  |
%%%     |    ·→x|
%%%     |         |
%%%     P3--------P4
R=[cos(phi), sin(phi);
 -sin(phi) , cos(phi)];%%%旋转矩阵
 
qicijuzhen=[0,0,1];
O=[x;z];
O1=[0;0];
O2=[0;0];

T=[R,     O;
    qicijuzhen];%%%基座的转换矩阵

%%%%顶点位置

Pforce=T*[0;-trunk_center_pos;1];

%%%%重心位置
PG=[x,z];

%%%%雅可比

Jforce=jacobian(Pforce(1:2),q)



%%%%速度


V_com=[dx,dz];

W = dphi*j;        % link  2 angular velocity

% W_m1 = k_r1 * dq1 * k;         % motor 1 angular velocity
% W_m2 = (dq1 + k_r2 * dq2) * k; % motor 2 angular velocity

% inertia momentum w.r.t. inertial frame(a.k.a. world frame)
% diag_inertia = @(input)( diag([0, 0, input]) ); % convert into diagonal inertia matrix
I_w = R * I * R';       % link   1 inertia

% I_m1_w = R_m1 * diag_inertia(I_m1) * R_m1';   % motor  1 inertia
% I_m2_w = R_m2 * diag_inertia(I_m2) * R_m2';   % motor  2 inertia



% F1=[F1x,F1y,0]';
% F2=[F2x,F2y,0]';
% F3=[F3x,F3y,0]';
% F4=[F4x,F4y,0]';
Fforce=[Fforcex,Fforcez]';
tal=[F1,F2,tal1];
% kinematic energy
KE = 0.5 * M * dot(V_com,V_com) + 0.5 * W' * I_w * W ;

% potential energy
PE = M * g * PG(2);

%% Lagrangian derivation
L = simplify(KE - PE);
DL_Ddq = jacobian(L,qd);
% dDL_Ddq_dt - DL_Dq' = Tau
DL_Dq = jacobian(L,q);

dDL_Ddq_dt = jacobian(DL_Ddq, q) * qd + jacobian(DL_Ddq, qd) *qdd;

%eqn = simplify(dDL_Ddq_dt - DL_Dq' -tal'- Jforce'* Fforce);
eqn = simplify(dDL_Ddq_dt - DL_Dq' -tal'- Jforce'* Fforce);

[MM, FF] = equationsToMatrix(eqn, qdd); % convert equation to : MM * ddq = FF
MM;
FF;
[aa, cc] = equationsToMatrix(eqn, tal)
