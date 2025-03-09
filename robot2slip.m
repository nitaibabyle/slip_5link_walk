function [left_length,left_angle,right_length,right_angle]=robot2slip(z0,slip)

x = z0(1);
z= z0(2);

base_pos=[x;0;z];
[base_frame,top_pos,left_knee_pos,right_knee_pos,left_foot_pos,right_foot_pos]=kinematic(z0,1,slip);

leftfoot_pos_wrt_base=base_pos-left_foot_pos;
rightfoot_pos_wrt_base=base_pos-right_foot_pos;

left_length=norm(leftfoot_pos_wrt_base);
right_length=norm(rightfoot_pos_wrt_base);
left_angle=atan(-leftfoot_pos_wrt_base(1)/leftfoot_pos_wrt_base(3));
right_angle=atan(-rightfoot_pos_wrt_base(1)/rightfoot_pos_wrt_base(3));
