%objective function
 function obj = ObjectiveFun(L) 
 %Initiation variables to call inverse Static function [Invstatic()]
 %Our trajectory is a circle with radius r = 50mm and centre at (100,300)
 %rotating at angular speed w0 = (pi/2)rad/sec
 %These assumptions need to modify according the condition
    syms l1 l2 l5 time 
    r = 50;
    p = 400;
    w0 = 3.14/2;
    q = 100;
    lb = [200,230,100];
    ub = [450,675,225];
    L0 = (lb + ub)./2;

  %Defining trajectory  
    
    xF = q+r*cos(w0*time + 3.14);
    yF = p-r*sin(w0*time + 3.14);
    %Calling inverse static function 
    %T1 = torque of 1st motor
    %T2 = torque of 2nd motor
    %theta1 = angle from figure which is position of motor 1
    %theta2 = angle from figure which is position of motor 2
    [T1,T2,theta1,theta2] = Invstatic(l1,l2,l5, xF, yF);
   
    %finding angular speed of motor 1 and 2
    w1Time = diff(theta1 , time);
    w2Time = diff(theta2 , time);
    
    %defining sub-objective function
    f1Time = T1;
    f2Time = T2;
    f3Time = w1Time;
    f4Time = w2Time;
    
    %Negation as fminbnd gives min value of the function so by negation
    %negative value becomes positive value
    f1TimeN =(-1)*f1Time;
    f2TimeN = (-1)*f2Time;
    f3TimeN = (-1)*f3Time;
    f4TimeN = (-1)*f4Time;
    
    %Substituting the L to get double value in fminbnd
    f1TimeL = subs(f1TimeN,[l1,l2,l5],L);
    f2TimeL = subs(f2TimeN,[l1,l2,l5],L);
    f3TimeL = subs(f3TimeN,[l1,l2,l5],L);
    f4TimeL = subs(f4TimeN,[l1,l2,l5],L);
    
    %Solve to find extreame time positions
    Tin = 0;
    Tout = 4;
    [TimeM1,f1MAXN] = fminbnd(@(x) double(subs(f1TimeL,time,x)),Tin,Tout);  
    [TimeM2,f2MAXN] = fminbnd(@(x) double(subs(f2TimeL,time,x)),Tin,Tout);  
    [TimeM3,f3MAXN] = fminbnd(@(x) double(subs(f3TimeL,time,x)),Tin,Tout);  
    [TimeM4,f4MAXN] = fminbnd(@(x) double(subs(f4TimeL,time,x)),Tin,Tout); 
   
    
    %substituting result time value in previous equations
    f1L = subs(f1Time,time,TimeM1);
    f2L = subs(f2Time,time,TimeM2);
    f3L = subs(f3Time,time,TimeM3);
    f4L = subs(f4Time,time,TimeM4);
    f5L = (2*l1+2*l2+l5);
    %Substituting these functions which are in terms of symbolic variable
    %defined on line 9 with input L array. Also we need to convert these
    %function into type double for the input to the fmincon function
    f1 = double(subs(f1L,[l1,l2,l5],L));
    f2 = double(subs(f2L,[l1,l2,l5],L));
    f3 = double(subs(f3L,[l1,l2,l5],L));
    f4 = double(subs(f4L,[l1,l2,l5],L));
    f5 = double(subs(f5L,[l1,l2,l5],L));
    
    f10 = double(subs(f1L,[l1,l2,l5],L0));
    f20 = double(subs(f2L,[l1,l2,l5],L0));
    f30 = double(subs(f3L,[l1,l2,l5],L0));
    f40 = double(subs(f4L,[l1,l2,l5],L0));
    f50 = double(subs(f5L,[l1,l2,l5],L0));
    
    f1A = abs(f1)/abs(f10);
    f2A = abs(f2)/abs(f20);
    f3A = abs(f3)/abs(f30);
    f4A = abs(f4)/abs(f40);
    f5A = abs(f5)/abs(f50);
    
    %Considering all the sub-objective functions as equaly weighted
    obj = f1A + f2A + f3A + f4A + f5A;
    
end