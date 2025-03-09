function zdot=rhs_3link(t,z,slip)   

l1=slip.l1;
l2=slip.l2;
l3=slip.l3;
m1=slip.m1;
m2=slip.m2;
m3=slip.m3;

a1=slip.a1;
a2=slip.a2;
a3=slip.a3;
g=slip.g;
I1_3=slip.I1(3,3);
I2_3=slip.I2(3,3);
I3_3=slip.I3(3,3);
leg=slip.leg;
kp=slip.kp;
kd=slip.kd;

q1 = z(5);  
q2 = z(7);
q3 = z(9);
q=[q1;q2;q3];
qd1=z(6);
qd2=z(8);
qd3=z(10);
qd=[qd1;qd2;qd3];


%以肢体重心为slip质点
% J=[- l2*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - l1*sin(q1) - a3*(cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))), - l2*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - a3*(cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))), -a3*(cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)));
%      l2*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + l1*cos(q1) + a3*(cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))),   l2*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + a3*(cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))),  a3*(cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))];
%以整体质心为slip质点
J=[-(m2*(a2*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + l1*sin(q1)) + m3*(l2*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + l1*sin(q1) + a3*(cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))) + a1*m1*sin(q1))/(m1 + m2 + m3), -(m3*(l2*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + a3*(cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))) + a2*m2*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))/(m1 + m2 + m3), -(a3*m3*(cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))))/(m1 + m2 + m3);
     (m2*(a2*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + l1*cos(q1)) + m3*(l2*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + l1*cos(q1) + a3*(cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))) + a1*m1*cos(q1))/(m1 + m2 + m3),  (m3*(l2*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + a3*(cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))) + a2*m2*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))/(m1 + m2 + m3),  (a3*m3*(cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))))/(m1 + m2 + m3)];
 


zzz=[q1,qd1,q2,qd2,q3,qd3];
[a,b,PG3,PG]=COM_kinematic(zzz,0,slip);
direction=[PG(1)/norm(PG(1:2));PG(2)/norm(PG(1:2))];
f=kp*(leg-norm(PG(1:2))).*direction;
tal = J'*f;

phi=q1+q2+q3;
dphi=qd1+qd2+qd3;
pitch=(phi-pi/2)*-50+dphi*-10;
tal1 = tal(1);
tal2 = tal(2)-kd*qd2; 
tal3 = tal(3)+pitch-kd*qd3; 
% tal1 = tal(1);
% tal2 = tal(2); 
% tal3 = tal(3); 
xxxx=J\[3;-2];

M11 = I1_3 + I2_3 + I3_3 + a1^2*m1 + a2^2*m2 + a3^2*m3 + l1^2*m2 + l1^2*m3 + l2^2*m3 + 2*a3*l1*m3*cos(q2 + q3) + 2*a2*l1*m2*cos(q2) + 2*a3*l2*m3*cos(q3) + 2*l1*l2*m3*cos(q2);
M12 = m2*a2^2 + l1*m2*cos(q2)*a2 + m3*a3^2 + 2*m3*cos(q3)*a3*l2 + l1*m3*cos(q2 + q3)*a3 + m3*l2^2 + l1*m3*cos(q2)*l2 + I2_3 + I3_3;
M13 = I3_3 + a3^2*m3 + a3*l1*m3*cos(q2 + q3) + a3*l2*m3*cos(q3);
M21 = m2*a2^2 + l1*m2*cos(q2)*a2 + m3*a3^2 + 2*m3*cos(q3)*a3*l2 + l1*m3*cos(q2 + q3)*a3 + m3*l2^2 + l1*m3*cos(q2)*l2 + I2_3 + I3_3;
M22 = m2*a2^2 + m3*a3^2 + 2*m3*cos(q3)*a3*l2 + m3*l2^2 + I2_3 + I3_3;
M23 = m3*a3^2 + l2*m3*cos(q3)*a3 + I3_3;
M31 =I3_3 + a3^2*m3 + a3*l1*m3*cos(q2 + q3) + a3*l2*m3*cos(q3);
M32 =m3*a3^2 + l2*m3*cos(q3)*a3 + I3_3;
M33 =m3*a3^2 + I3_3;

RHS1 = tal1 - a2*g*m2*cos(q1 + q2) - g*l2*m3*cos(q1 + q2) - a1*g*m1*cos(q1) - g*l1*m2*cos(q1) - g*l1*m3*cos(q1) - a3*g*m3*cos(q1 + q2 + q3) + a3*l1*m3*qd2^2*sin(q2 + q3) + a3*l1*m3*qd3^2*sin(q2 + q3) + a2*l1*m2*qd2^2*sin(q2) + a3*l2*m3*qd3^2*sin(q3) + l1*l2*m3*qd2^2*sin(q2) + 2*a3*l1*m3*qd1*qd2*sin(q2 + q3) + 2*a3*l1*m3*qd1*qd3*sin(q2 + q3) + 2*a3*l1*m3*qd2*qd3*sin(q2 + q3) + 2*a2*l1*m2*qd1*qd2*sin(q2) + 2*a3*l2*m3*qd1*qd3*sin(q3) + 2*a3*l2*m3*qd2*qd3*sin(q3) + 2*l1*l2*m3*qd1*qd2*sin(q2);
RHS2 = tal2 - a2*g*m2*cos(q1 + q2) - g*l2*m3*cos(q1 + q2) - a3*g*m3*cos(q1 + q2 + q3) - a3*l1*m3*qd1^2*sin(q2 + q3) - a2*l1*m2*qd1^2*sin(q2) + a3*l2*m3*qd3^2*sin(q3) - l1*l2*m3*qd1^2*sin(q2) + 2*a3*l2*m3*qd1*qd3*sin(q3) + 2*a3*l2*m3*qd2*qd3*sin(q3);
RHS3 = tal3 - a3*g*m3*cos(q1 + q2 + q3) - a3*l1*m3*qd1^2*sin(q2 + q3) - a3*l2*m3*qd1^2*sin(q3) - a3*l2*m3*qd2^2*sin(q3) - 2*a3*l2*m3*qd1*qd2*sin(q3);
M=[M11 M12 M13;M21 M22 M23;M31 M32 M33];
RHS=[RHS1 ; RHS2 ; RHS3];
qdd= M \ RHS;
zdot=[0 0 0 0 qd(1) qdd(1) qd(2) qdd(2) qd(3) qdd(3)]';

