function [P_E1, P_E2]=kinematic1(z,t,slip)
q1 = z(1);                          
qd1 = z(2);                          
q2 = z(3);                         
qd2 = z(4);

l1=slip.l1;
l2=slip.l2;
% i = [1; 0; 0];  j = [0; 1; 0];  k = [0; 0; 1];
% q = [q1; q2];
% dq = [qd1; qd2];
% 
% P_E1 = l1 * sin(q1) * i + l1 * -cos(q1) * j; % position of E1
% P_E2 = (l1 * sin(q1) + l2 * sin(q1 - q2)) * i + (l1 * -cos(q1) + l2 * -cos(q1 - q2)) * j; % position of E2

R1=[sin(q1), cos(q1), 0;
    -cos(q1), sin(q1), 0;
          0,       0, 1];
O1=[0;0;0];
aaaa=[0,0,0,1];
R2=[ cos(-q2), -sin(-q2), 0;
    sin(-q2), cos(-q2), 0;
           0,       0, 1];    
R12=R1*R2;
O2=[l1;0;0];
% R_com_1=a1;
% R_com_2=a2;
T1=[R1,O1;
    aaaa];
T2=[R2,O2;
    aaaa];
T12=T1*T2;
P_E1=T1*[l1;0;0;1];
P_E2=T12*[l2;0;0;1];