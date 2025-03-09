slip.l1=1;
slip.l2=1;
slip.l3=1;
slip.l4=1;
slip.l5=1;
q1    = pi*70/180; %angle made by link1 with vertical
q2    = pi*30/180 ;      %angle made by link2 with vertical
q3      =-pi*1/18;
q4      =-pi*8/9;
q5      =-pi*30/180;
z0=[q1 q2 q3 q4 q5];

[P1,P2,P3,P4,P5]=kinematic(z0,slip);
figure(1)
    window_xmin = -3*slip.l1; window_xmax = 3*slip.l1;
    window_ymin = -3*slip.l1; window_ymax = 3*slip.l1;
    plot(0,0,'ko','MarkerSize',3); %pivot point
    line([0 P1(1)],[0 P1(2)],'Linewidth',4,'Color',[0.8 0 0]);% first pendulum
    line([P1(1) P2(1)],[P1(2) P2(2)],'Linewidth',4,'Color',[0.8 0 0]);% second pendulum
    line([P2(1) P3(1)],[P2(2) P3(2)],'Linewidth',4,'Color',[0 0 0]);
line([P2(1) P4(1)],[P2(2) P4(2)],'Linewidth',4,'Color',[0 0.8 0.8]);
line([P4(1) P5(1)],[P4(2) P5(2)],'Linewidth',4,'Color',[0 0.8 0.8]);
    axis('equal')
    axis on
    axis([window_xmin window_xmax window_ymin window_ymax])
