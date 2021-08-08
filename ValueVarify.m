%lets find equations 
    r = 50;
    p = 400;
    w0 = 3.14/2;
    %syms l1 l2 l5 time
    syms time
    %l3 = l2;
    %l4 = l1;
    %l6 = 0;
    l5 = 116.1291;
    l1 =248.0009 ;
    l2 = 512.7047;
    q = 60;
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
    f1Time = w1Time;
    f2Time = w2Time;
    f3Time = T1;
    f4Time = T2;
    
    f1TimeN = (-1)*abs(f1Time);
    f2TimeN = (-1)*abs(f2Time);
    f3TimeN = (-1)*abs(f3Time);
    f4TimeN = (-1)*abs(f4Time);
   
  
    %Solve to find extreame time positions
    Tin = -1;
    Tout = 2.5;
    [TimeM1,f1MAXN] = fminbnd(@(x) double(subs(f1TimeN,time,x)),Tin,Tout);  
    [TimeM2,f2MAXN] = fminbnd(@(x) double(subs(f2TimeN,time,x)),Tin,Tout);  
    [TimeM3,f3MAXN] = fminbnd(@(x) double(subs(f3TimeN,time,x)),Tin,Tout);  
    [TimeM4,f4MAXN] = fminbnd(@(x) double(subs(f4TimeN,time,x)),Tin,Tout);  
    
   
    f1MAX = double(subs(f1Time,time,TimeM1));
    f2MAX = double(subs(f2Time,time,TimeM2));
    f3MAX = double(subs(f3Time,time,TimeM3));
    f4MAX = double(subs(f4Time,time,TimeM4));
   
    %plot theta input Vs Time
figure(1);
axis([-5 5 -1 1]);
hold on
fplot(time, (f1Time) , 'r');
hold on
fplot(time, (f2Time) , 'b');

plot(double(TimeM1), double(f1MAX), 'g*');
hold on
plot(double(TimeM2), double(f2MAX), 'g*');
hold on
legend('f1Time','f2Time','Maxf1','Maxf2')
ylabel('angular Velocity input')
xlabel('Time')
title('velocity(rad/sec)')
hold off

%plot w input Vs Time
figure(2);
axis([-5 5 -50000 50000]);
hold on
fplot(time, f3Time , 'r');
hold on
fplot(time, f4Time , 'b');
hold on
plot(double(TimeM3), double(f3MAX), 'g*');
hold on
plot(double(TimeM4), double(f4MAX), 'g*');
legend('f3Time','f4Time','Maxf3','Maxf4')
ylabel('angular velocity input')
xlabel('Time')
title('Torque(Nm)')
hold off




