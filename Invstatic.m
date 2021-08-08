function [T1,T2,theta1,theta2] = Invstatic(l1,l2,l5, xF, yF)

    l3 = l2;
    l4 = l1;
    
    [theta1, theta2, phi1, phi2] = InvKin(l1,l2,l3,l4,l5, xF, yF);
    
    %Fx = frictional force assumed as static limiting friction
    %and u = o.5 W = 30*9.81;
    %Fy = force due to weight i.e. W/4
    W = 30*9.81;      
    u = 0.5;
    Fx = u*W;
    Fy = W/4;
    %Jecobian matrix
    J11 = ((-1*l1*sin(theta1)) + ((l2)*sin(phi1)*l1*sin(phi2-theta1))) / (l2*sin(phi2-phi1));
    
    J12 = (-1*(l2)*sin(phi1)*l4*sin(phi2-theta2)) / (l2*sin(phi2-phi1));
    
    J21 = ((l1*cos(theta1)) - ((l2)*cos(phi1)*l1*sin(phi2-theta1))) / (l2*sin(phi2-phi1));
    
    J22 = ((l2)*cos(phi1)*l4*sin(phi2-theta2)) / (l2*sin(phi2-phi1));
    
    Jacob = [J11 J12 ; J21 J22];
    
    T = Jacob'*[Fx;Fy];
    T1 = T(1);
    T2 = T(2);
    %Torque value
    
    
end