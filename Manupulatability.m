L1 =1;
L2 =1;
q1 = 0;
q2 = 90;
J = [-L1*sind(q1)-L2*sind(q1+q2) -L2*sind(q1+q2);
      L1*cosd(q1)+L1*cosd(q1+q2) L2*cosd(q1+q2)];
JJ_t = J*J';
det(JJ_t);
