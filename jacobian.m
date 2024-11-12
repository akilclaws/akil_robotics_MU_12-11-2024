function [x,y] = jacobian(l1,l2,theta1,theta2,derivativexy)
a=[-l1*sin(theta1)-l2*sin(theta1+theta2) -l2*sin(theta1+theta2);l1*cos(theta1)+l2*cos(theta1+theta2) l2*cos(theta1+theta2)];
x= inv(a)*derivativexy;
y=det(a);
end