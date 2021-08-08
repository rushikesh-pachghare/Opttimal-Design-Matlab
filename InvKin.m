%Inverse Kinematics
function [theta1, theta2, phi1, phi2] = InvKin(l1,l2,l3,l4,l5, xF, yF)
    l6 = 0;
     %syms time
    U = 2*l1*yF;
    V = 2*l1*xF;
    W = (l2+l6)^2 -l2^2 - xF^2 - yF^2;
    
    theta1 = 2*atan((U + (U^2 +V^2 - W^2)^(1/2))/(V-U));
    
    phi1 = acos((l1^2 +(l6 +l2)^2 -(xF^2 + yF^2))/(2*l1*(l2 +l6))) +theta1 -pi;  
    
    u = 2*((l4*l6*sin(phi1)) -(l4*yF));
    v= 2*((l4*l5)-(l4*xF)+(l4*l6*cos(phi1)));
    w = xF^2 +yF^2 - (2*xF*(l5 +(l6*cos(phi1)))) + (2*l5*l6*cos(phi1)) - (2*l6*sin(phi1)*yF) - l3^2 + l4^2 + l5^2 + l6^2; 
    
    theta2 = 2*atan((u + (u^2 +v^2 - w^2)^(1/2))/(v-u));
    
    phi2 = asin((yF - (l4*sin(theta2)) - (l4*sin(phi1)))/l3);
    


end

