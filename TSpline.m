function [S,Sd,Sdd]=TSpline(p0,v0,t0,p1,t1,p2,v2,t2,t)
% p0=1;
% v0=0;
% t0=0;%第一个点的时间，位置，速度
% p1=5;
% t1=2;%第二个点的时间，位置
% p2=1;
% v2=0;
% t2=5;%第三个点的时间，位置，速度
%step=0.1;
R=[1,    t0,   t0*t0,  t0*t0*t0,       0,        0,       0,        0;
   0,     1,    2*t0,   3*t0*t0,       0,        0,       0,        0;
   1,    t1,   t1*t1,  t1*t1*t1,       0,        0,       0,        0;
   0,     0,       0,         0,       1,       t2,   t2*t2, t2*t2*t2;
   0,     0,       0,         0,       0,        1,    2*t2,  3*t2*t2;
   0,     0,       0,         0,       1,       t1,   t1*t1, t1*t1*t1;
   0,     1,    2*t1,   3*t1*t1,       0,       -1,   -2*t1, -3*t1*t1;
   0,     0,       2,      6*t1,       0,        0,      -2,    -6*t1];
p=[p0,v0,p1,p2,v2,p1,0,0]';
p=R\p;
i=t;
if t<=t1
    S=p(1)+p(2)*i+p(3)*i*i+p(4)*i*i*i;
    Sd=p(2)+2*p(3)*i+3*p(4)*i*i;
    Sdd=2*p(3)+6*p(4)*i;
elseif t<t2
    S=p(5)+p(6)*i+p(7)*i*i+p(8)*i*i*i;
    Sd=p(6)+2*p(7)*i+3*p(8)*i*i;
    Sdd=2*p(7)+6*p(8)*i;
else
    S=p2;
    Sd=p(2)+2*p(3)*i+3*p(4)*i*i;
    Sdd=2*p(3)+6*p(4)*i;
end


    