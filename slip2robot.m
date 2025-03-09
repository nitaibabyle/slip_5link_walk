function [leg_hip_angle_left,leg_knee_angle_left,leg_hip_angle_right,leg_knee_angle_right]=slip2robot(z0,left_length,left_angle,right_length,right_angle,slip)

thigh_length=slip.thigh_length;
shank_length=slip.shank_length;

leg_knee_angle_left=pi-acos((thigh_length^2+shank_length^2-left_length^2)/(2*shank_length*thigh_length));
leg_knee_angle_right=pi-acos((thigh_length^2+shank_length^2-right_length^2)/(2*thigh_length*shank_length));

leg_hip_angle_left=+z0(3)+left_angle+acos((thigh_length^2+left_length^2-shank_length^2)/(2*thigh_length*left_length));
leg_hip_angle_right=+z0(3)+right_angle+acos((thigh_length^2+right_length^2-shank_length^2)/(2*thigh_length*right_length));

if leg_knee_angle_left<10*pi/180
    leg_knee_angle_left=10*pi/180;
end
if leg_knee_angle_right<10*pi/180
    leg_knee_angle_right=10*pi/180;
end
