function [J_left_foot,J_right_foot]=jacobian_of_foot(z0,flag,slip)
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

% 
% J_left_foot=jacobian(P_end_shank_left(1:3)-[x;0;z],q(4:end))
% J_right_foot=jacobian(P_end_shank_right(1:3)-[x;0;z],q(4:end))

J_left_foot =[  thigh_length*(cos(leg_hip_angle_left)*cos(phi) + sin(leg_hip_angle_left)*sin(phi)) + shank_length*(cos(leg_knee_angle_left)*(cos(leg_hip_angle_left)*cos(phi) + sin(leg_hip_angle_left)*sin(phi)) - sin(leg_knee_angle_left)*(cos(leg_hip_angle_left)*sin(phi) - cos(phi)*sin(leg_hip_angle_left))), -shank_length*(cos(leg_knee_angle_left)*(cos(leg_hip_angle_left)*cos(phi) + sin(leg_hip_angle_left)*sin(phi)) - sin(leg_knee_angle_left)*(cos(leg_hip_angle_left)*sin(phi) - cos(phi)*sin(leg_hip_angle_left)));
    0,                                                                                                                                                                                                               0;
    - thigh_length*(cos(leg_hip_angle_left)*sin(phi) - cos(phi)*sin(leg_hip_angle_left)) - shank_length*(cos(leg_knee_angle_left)*(cos(leg_hip_angle_left)*sin(phi) - cos(phi)*sin(leg_hip_angle_left)) + sin(leg_knee_angle_left)*(cos(leg_hip_angle_left)*cos(phi) + sin(leg_hip_angle_left)*sin(phi))),  shank_length*(cos(leg_knee_angle_left)*(cos(leg_hip_angle_left)*sin(phi) - cos(phi)*sin(leg_hip_angle_left)) + sin(leg_knee_angle_left)*(cos(leg_hip_angle_left)*cos(phi) + sin(leg_hip_angle_left)*sin(phi)))];
 
J_right_foot =[  thigh_length*(cos(leg_hip_angle_right)*cos(phi) + sin(leg_hip_angle_right)*sin(phi)) + shank_length*(cos(leg_knee_angle_right)*(cos(leg_hip_angle_right)*cos(phi) + sin(leg_hip_angle_right)*sin(phi)) - sin(leg_knee_angle_right)*(cos(leg_hip_angle_right)*sin(phi) - cos(phi)*sin(leg_hip_angle_right))), -shank_length*(cos(leg_knee_angle_right)*(cos(leg_hip_angle_right)*cos(phi) + sin(leg_hip_angle_right)*sin(phi)) - sin(leg_knee_angle_right)*(cos(leg_hip_angle_right)*sin(phi) - cos(phi)*sin(leg_hip_angle_right)));
    0,                                                                                                                                                                                                                     0;
    - thigh_length*(cos(leg_hip_angle_right)*sin(phi) - cos(phi)*sin(leg_hip_angle_right)) - shank_length*(cos(leg_knee_angle_right)*(cos(leg_hip_angle_right)*sin(phi) - cos(phi)*sin(leg_hip_angle_right)) + sin(leg_knee_angle_right)*(cos(leg_hip_angle_right)*cos(phi) + sin(leg_hip_angle_right)*sin(phi))),  shank_length*(cos(leg_knee_angle_right)*(cos(leg_hip_angle_right)*sin(phi) - cos(phi)*sin(leg_hip_angle_right)) + sin(leg_knee_angle_right)*(cos(leg_hip_angle_right)*cos(phi) + sin(leg_hip_angle_right)*sin(phi)))];
 


