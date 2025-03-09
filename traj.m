function angle_des=traj(startpoint,endpoint,time,omega)
% if (time>(1*pi/(8*omega)))&&(time<(5*pi/(8*omega)))
%     angle_des = ((startpoint+endpoint)/2)-(cos((2*omega)*(time-(pi/(4*omega)-(pi/(8*omega)))))*(endpoint-startpoint)/2);
% end
% if time<=(1*pi/(8*omega))
%     angle_des = startpoint;
% end
% if time>=(5*pi/(8*omega))
%     angle_des= endpoint;
% end
p0=startpoint;v0=0;t0=0;p1=(endpoint+startpoint+0.1)/2;t1=0.15;p2=endpoint;t2=0.25;v2=0;
[S,Sd,Sdd]=TSpline(p0,v0,t0,p1,t1,p2,v2,t2,time);
angle_des=S;
%%%这是最初版本
% if (time>(pi/(5*omega)))&&(time<(4*pi/(5*omega)))
%     angle_des = ((startpoint+endpoint)/2)-(cos(((5/3)*omega)*(time-(pi/(5*omega))))*(endpoint-startpoint)/2);
% end
% if time<=(pi/(5*omega))
%     angle_des = startpoint;
% end
% if time>=(4*pi/(5*omega))
%     angle_des= endpoint;
% end