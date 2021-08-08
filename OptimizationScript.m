%optimization main

%Coordinates of the end-Effector from path(Circle)...pt(100,300) being centre
%xF = r*cos(w0*time + pi) + 100;
%yF = r*sin(w0*time + pi) - 300;
%==========================================================================
%                       START
%This is the  main script which call other function and perform
%optimization. We are using fmincon() function for this purpose. it takes
%values such as lower and upper boundary, linear and non linear constrains
%and also many things

%Our problem have some non linear constrains and boundaries

%lower and upper boundry
lb = [200,230,100];
ub = [450,675,225];

%Other prrameters
A = [];
b = [];
Aeq = [];
beq = [];

%Initial value
L0 = (lb + ub)./2;

%Calling the function
Obj = @ObjectiveFun;

% Calling Non linear constrain function
const = @constrainfunc;%Constrain function which is void as of now

%Optimization obtions like algorithm selection(here 'active-set') and
%displaying itteration
options = optimoptions('fmincon','Display','iter');

%calling the optimization command
%L = is an array of index 3...L(1) to elements of L represent lengths of
%the links
L = fmincon(Obj,L0,A,b,Aeq,beq,lb,ub,const,options);
F = Obj(L);


