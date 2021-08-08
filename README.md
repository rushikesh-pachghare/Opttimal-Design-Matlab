#Introduction
A five-bar mechanism is designed and optimized as a parallel leg of a quadruped. Inverse kinematics and statics are analyzed. 
This paper aims to optimize the link lengths of the five-bar mechanism as a leg of the quadruped robot in such a manner that
the maximum torque and maximum angular velocity of the motors should be minimum. This optimization is done considering that  
the robot’s payload (weight included) is 30kg and 0.4m/s walking speed. And an ‘interior-point algorithm' is used with the
MATLAB optimization toolbox to obtain the solution of the given optimization problem. In addition, Simulink simulation is
also done to support the results of the optimization process.

#RUN the OptimizationScript.m file which will call all the other files and function to calculate optimal values of length of the link. 
#You can verify these values by putting these values in ValueVerify.m and runnung that file
#Matlab Optimization toolbox is used in this project
