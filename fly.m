function zdot = fly(t,z,slip)
x=z(1);
xd=z(2);
y=z(3);
yd=z(4);
q1=z(5);
qd1=z(6);
q2=z(7);
qd2=z(8);
q3=z(9);
qd3=z(10);

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

start_angle=slip.final;

error=q1-start_angle+pi/2-pi/22;

tal1=0;
tal2=0;
MM =[I1_3 + I2_3 + a1^2*m2 + a2^2*m1 + l1^2*m2 + l2^2*m1 + l2^2*m2 + 2*l2^2*mmm + 2*l2^2*mmm*cos(q2) - 2*a1*l1*m2 - 2*a2*l2*m1 - 2*a1*l2*m2*cos(q2) + 2*l1*l2*m2*cos(q2), I2_3 + a1^2*m2 + l1^2*m2 + l2^2*mmm + l2^2*mmm*cos(q2) - 2*a1*l1*m2 - a1*l2*m2*cos(q2) + l1*l2*m2*cos(q2);
                                                          I2_3 + a1^2*m2 + l1^2*m2 + l2^2*mmm + l2^2*mmm*cos(q2) - 2*a1*l1*m2 - a1*l2*m2*cos(q2) + l1*l2*m2*cos(q2),                                                          m2*a1^2 - 2*m2*a1*l1 + m2*l1^2 + mmm*l2^2 + I2_3];

FF =[tal1 + a1*g*m2*cos(q1 + q2) - g*l1*m2*cos(q1 + q2) - g*l2*mmm*cos(q1 + q2) + l2^2*mmm*qd2^2*sin(q2) + a2*g*m1*cos(q1) - g*l2*m1*cos(q1) - g*l2*m2*cos(q1) - g*l2*mmm*cos(q1) - a1*l2*m2*qd2^2*sin(q2) + l1*l2*m2*qd2^2*sin(q2) + 2*l2^2*mmm*qd1*qd2*sin(q2) - 2*a1*l2*m2*qd1*qd2*sin(q2) + 2*l1*l2*m2*qd1*qd2*sin(q2);
                                                                                                                                                                tal2 + a1*g*m2*cos(q1 + q2) - g*l1*m2*cos(q1 + q2) - g*l2*mmm*cos(q1 + q2) - l2^2*mmm*qd1^2*sin(q2) + a1*l2*m2*qd1^2*sin(q2) - l1*l2*m2*qd1^2*sin(q2)];

qdd= MM \ FF;

zdot = [z(2) 0 z(4) -slip.g 0 0  0 0 0 0]';

