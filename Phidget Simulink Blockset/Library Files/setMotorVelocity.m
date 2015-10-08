% MATLAB Function demonstrating how to set output states
function setMotorVelocity(speed)
speedLeft = double(speed(1));
speedRight= double(speed(2));
global handleLeft;
global handleRight;

calllib('phidget21','CPhidgetMotorControl_setVelocity', handleLeft,0, speedLeft);
calllib('phidget21','CPhidgetMotorControl_setVelocity', handleRight,0, speedRight);


