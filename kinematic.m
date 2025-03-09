function [PB, P_E1, P_E2, P_E3, P_E4, P_E5]=kinematic(z,t,slip)
x = z(1);
zz= z(2);
phi = z(3);
qlh = z(4);
qlk = z(5);
qrh = z(6);
qrk = z(7);

trunk_length=slip.trunk_length;
thigh_length=slip.thigh_length;
shank_length=slip.shank_length;

Rtrunk=[cos(phi),  0, sin(phi);
               0,  1,         0;
        -sin(phi),  0,  cos(phi)]; %%%基座旋转矩阵

Rleft_thigh2torso=[cos(-qlh),  0, sin(-qlh);
                          0,  1,         0;
                   -sin(-qlh),  0,  cos(-qlh)]; %%%左大腿rotation matrix

Rleft_shank2thigh=[cos(qlk),  0, sin(qlk);
                          0,  1,         0;
                   -sin(qlk),  0,  cos(qlk)]; %%%左小腿rotation matrix    

Rright_thigh2torso=[cos(-qrh),  0, sin(-qrh);
                           0,  1,         0;
                    -sin(-qrh),  0,  cos(-qrh)]; %%%右大腿rotation matrix

Rright_shank2thigh=[cos(qrk),  0, sin(qrk);
                           0,  1,         0;
                    -sin(qrk),  0,  cos(qrk)]; %%%右小腿rotation matrix   

qicijuzhen=[0,0,0,1];

O=[x;0;zz];%%%基座的原点位置
Ttrunk=[Rtrunk,     O;
           qicijuzhen];%%%上身相对世界坐标系的Trans矩阵
P_end_trunk=Ttrunk*[0;0;trunk_length;1];%%%上身顶点的位置

O1=[0;0;0];%%%大腿相对于上身的Trans矩阵
T_left_thigh = [Rleft_thigh2torso,     O1;
                             qicijuzhen];%%%
T_right_thigh = [Rright_thigh2torso,     O1;
                               qicijuzhen];%%%
P_end_thigh_left=Ttrunk*T_left_thigh*[0;0;-thigh_length;1];%%%大腿末端位置
P_end_thigh_right=Ttrunk*T_right_thigh*[0;0;-thigh_length;1];  

O2=[0;0;-thigh_length];%%%小腿相对于大腿的Trans矩阵                     
T_left_shank = [Rleft_shank2thigh,     O2;
                               qicijuzhen];
T_right_shank = [Rright_shank2thigh,     O2;
                                 qicijuzhen];                         
P_end_shank_left = Ttrunk*T_left_thigh*T_left_shank*[0;0;-shank_length;1];%%%足端的位置
P_end_shank_right = Ttrunk*T_right_thigh*T_right_shank*[0;0;-shank_length;1]; 

PB=O;

P_E1 = P_end_trunk(1:3);
P_E2 = P_end_thigh_left(1:3);
P_E3 = P_end_thigh_right(1:3);
P_E4 = P_end_shank_left(1:3);
P_E5 = P_end_shank_right(1:3);





