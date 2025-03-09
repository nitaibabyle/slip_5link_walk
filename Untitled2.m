
angle=[];
omega=pi/0.4;
time=linspace(0,1,100);
startpoint=pi/6;
endpoint=-pi/6;
for i=1:100
angle_des(i)=traj(startpoint,endpoint,time(i),omega);
end
figure(1);
plot(time,angle_des);