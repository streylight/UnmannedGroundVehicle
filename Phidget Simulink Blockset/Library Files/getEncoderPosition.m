% MATLAB Function demonstrating how to get encoder position
function Enc = getEncoderPosition(Enc)

global handleLeft;
global handleRight;
Enc = zeros(1,2);
dataptr = libpointer('int32Ptr',0);
calllib('phidget21','CPhidgetMotorControl_getEncoderPosition', handleLeft,0,dataptr);
Enc(1) = get(dataptr, 'Value');
calllib('phidget21','CPhidgetMotorControl_getEncoderPosition', handleRight,0,dataptr);
Enc(2) = get(dataptr, 'Value');

