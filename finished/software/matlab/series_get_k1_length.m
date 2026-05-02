function K = series_get_k1_length(leg_length,ak)
   
    syms x(t) T R Iw mw L theta(t) mp g Ip 
    syms f1 f2 d_theta d_x theta0 x0  

    lw=polyval(ak,leg_length);  %求ak多项式在leg_length的值    %摆杆质心高度（以轮轴为原点）
    lb=leg_length+0.0072;               %机体质心高度
    larm=leg_length+0.1;                %机械臂质心高度
    R1=0.049;                           %驱动轮半径
    mw1=0.294;                         %驱动轮质量
    marm=0.194*0.5;                         %机械臂质量
    mb=1.43*0.5 ;                      %机体质量
    ml=0.144;                          %摆杆质量
    mp1=mb+ml+marm;                        %杆质量+机体
    L1=(lb*mb+lw*ml+larm*marm)/mp1;           %总重心到驱动轮轴距离
    Iw1=mw1*R1^2;                     %驱动轮转动惯量
    Ip1=mb*(lb-L1)^2+ml*(L1-lw)^2+marm*(L1-larm)^2;     %摆杆转动惯量
   
    N = mp*diff(x + L*sin(theta),t,2);
    P = mp*g+mp*diff(L*cos(theta),t,2);

    eqn1 = diff(x,t,2) == (T -N*R)/(Iw/R + mw*R);
    eqn2 = Ip*diff(theta,t,2) == P*L*sin(theta)-N*L*cos(theta)-T;
   
    
    eqn10 = subs(subs(subs(subs(subs(subs(subs(eqn1,diff(theta,t,2),f1),diff(x,t,2),f2)),diff(theta,t),d_theta),diff(x,t),d_x),theta,theta0),x,x0);
    eqn20 = subs(subs(subs(subs(subs(subs(subs(eqn2,diff(theta,t,2),f1),diff(x,t,2),f2)),diff(theta,t),d_theta),diff(x,t),d_x),theta,theta0),x,x0);
  

    [f1,f2] = solve(eqn10,eqn20,f1,f2);
   
    A=subs(jacobian([d_theta,f1,d_x,f2],[theta0,d_theta,x0,d_x,]),[theta0,d_theta,d_x,T],[0,0,0,0]);
    A=subs(A,[R,L,mw,mp,Iw,Ip,g],[R1,L1,mw1,mp1,Iw1,Ip1,9.8]);
    A=double(A);
    B=subs(jacobian([d_theta,f1,d_x,f2],T),[theta0,d_theta,d_x,T],[0,0,0,0]);
    B=subs(B,[R,L,mw,mp,Iw,Ip,g],[R1,L1,mw1,mp1,Iw1,Ip1,9.8]);
    B=double(B);
    
    Q=diag([6000 15 200 50]);%theta d_theta x d_x %6000 15 200 50 
    R=500;                %T 
    
    K=lqr(A,B,Q,R);
  
end