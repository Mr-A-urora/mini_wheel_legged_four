
function lw = get_lw(leg_length)
syms l1 l2 l3 l4 l23 l5 f1 L0 lw
syms theta1 theta2 theta3 theta4 theta5 theta6
syms m1 m2 m3 m4 m5 m23 



l1 = 0.1204195; %%mm r5
l2 = 0.12168; %%mm r1
l3 = 0.12567; %%mm r3
l4 = 0.05872;  %%mm r2
l23 = 0.02958;  %%mm r4
 
m1=0.038;%l1*0.15^2;
m2=0.031;%l2*0.15^2;
m3=0.015;%l3*0.15^2;
m23=0.030;%l23*0.15^2;

theta1 = asin((leg_length^2+l2^2-l1^2)/(2*leg_length*l2));

l5=sqrt(l2^2+l4^2-2*l2*l4*cos(theta1+pi/4));
eqn1=l4/sin(theta2) == l5/sin(theta1+pi/4);
theta2 = simplify(solve(eqn1,theta2));
theta2=theta2(1);
eqn2=l3^2==l23^2 + l5^2 -2*l23*l5*cos(theta3);
theta3 = simplify(solve(eqn2,theta3));
theta3=theta3(2);
theta4=pi-theta1-theta2-theta3;
theta4=simplify(theta4);
theta5=pi-theta4;
theta6=asin(l5*sin(theta3)/l3)-theta4;
theta6=simplify(theta6);

xc2 = l2*sin(theta1)/2;
xc1 = l2*sin(theta1)+l1*sin(theta5)/2;
xc3 = -l4*sin(pi/4) + l3*sin(theta6)/2;
xc23 = l2*sin(theta1) - l23*sin(theta4)/2;
yc=(m1*xc1+m2*xc2+m3*xc3+m23*xc23)/(m1+m2+m3+m23);
yc=simplify(yc);

lw = double(leg_length-yc);

end
