% MATLAB Function demonstrating how to get encoder position
function Son = getSonarDistance(Son)
global handleLeft;
global handleRight;
Son = zeros(1,2);
dataptr = libpointer('int32Ptr',0);
calllib('phidget21','CPhidgetMotorControl_getSensorValue', handleLeft,0,dataptr);
Son(1) = get(dataptr, 'Value');
calllib('phidget21','CPhidgetMotorControl_getSensorValue', handleRight,0,dataptr);
Son(2) = get(dataptr, 'Value');
