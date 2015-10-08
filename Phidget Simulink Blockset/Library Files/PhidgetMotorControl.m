...........................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................% S-Function demonstrating how to load the Phidget library as well as creating, opening and attaching to the Phidget
function [sys,x0,str,ts] = PhidgetMotorControl(t,x,u,flag, serialLeft,serialRight)

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
   [sys,x0,str,ts]=mdlInitializeSizes(serialLeft,serialRight);
  
  %%%%%%%%%%  
  % Update %
  %%%%%%%%%%
  case 2,                                               
    sys = mdlUpdate(t,x,u);
    
  %%%%%%%%%%
  % Output %
  %%%%%%%%%%
  case 3,                                               
    sys = mdlOutputs(t,x,u);    
    
  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  otherwise
    error(['unhandled flag = ',num2str(flag)]);
end

%end sfundsc1

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes(serialLeft,serialRight)

global handleLeft;
global oldHandleLeft;
global handleRight;
global oldHandleRight;

if ~libisloaded('phidget21')
	warning off MATLAB:loadlibrary:TypeNotFound
    warning off MATLAB:loadlibrary:TypeNotFoundForStructure
    switch computer
        case 'PCWIN'
            [notfound,warnings]=loadlibrary('phidget21', 'phidget21Matlab_Windows_x86.h');
        case 'PCWIN64'
            [notfound,warnings]=loadlibrary('phidget21', 'phidget21Matlab_Windows_x64.h');
        case 'MAC'
        case 'MACI'
        case 'MACI64'
            [notfound,warnings]=loadlibrary('/Library/frameworks/Phidget21.framework/Versions/Current/Phidget21', 'phidget21matlab_unix.h', 'alias', 'phidget21');
        case 'GLNX86'
        case 'GLNXA64'
            [notfound,warnings]=loadlibrary('/usr/lib/libphidget21.so', 'phidget21matlab_unix.h', 'alias', 'phidget21');
    end
end

handleLeft = libpointer('int32Ptr');
handleRight = libpointer('int32Ptr');
calllib('phidget21', 'CPhidgetMotorControl_create', handleLeft);
calllib('phidget21', 'CPhidgetMotorControl_create', handleRight);
%if the open call fails, then either the Phidget is not attached to the
%computer, or the application has already attached to the Phidget(ie, even
%when a simulation is not running, a change in the block parameter will run
%mdlInitializeSizes
if calllib('phidget21', 'CPhidget_open', handleLeft, serialLeft) ~= 0
    %the Phidget may have been opened when the simulation was not running, 
    %so the old handle becomes the current handle
    handleLeft = oldHandleLeft    ;
end
if calllib('phidget21', 'CPhidget_open', handleRight, serialRight) ~= 0
    %the Phidget may have been opened when the simulation was not running, 
    %so the old handle becomes the current handle
    handleRight = oldHandleRight ;
end
%application takes the next 2500ms to attach to the Phidget
if calllib('phidget21', 'CPhidget_waitForAttachment', handleLeft, 2500) == 0
    disp('Attached to MotorControlLeft')
    
    %if the block parameters changes while the simulation is not running,
    %then we will need to make a copy of the handle
    oldHandleLeft = handleLeft;
    
else
    disp('Could not attach to MotorControlLeft')
end
if calllib('phidget21', 'CPhidget_waitForAttachment', handleRight, 2500) == 0
    disp('Attached to MotorControlRight')
    
    %if the block parameters changes while the simulation is not running,
    %then we will need to make a copy of the handle
    oldHandleRight = handleRight;
    
else
    disp('Could not attach to MotorControlRight')
end

sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;   
sizes.NumOutputs     = 0;
sizes.NumInputs      = 0;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [-1 0]; 
% end mdlInitializeSizes

%
%=======================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=======================================================================
%
function sys = mdlUpdate(t,x,u)
sys = [];
%end mdlUpdate

%
%=======================================================================
% mdlOutputs
% Return the output vector for the S-function
%=======================================================================
%
function sys = mdlOutputs(t,x,u)
sys = [];

%end mdlOutputs

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)
global handleLeft;
global handleRight;

% clean up
calllib('phidget21','CPhidgetMotorControl_setVelocity', handleLeft,0, 0);
calllib('phidget21','CPhidgetMotorControl_setVelocity', handleRight,0, 0);
calllib('phidget21', 'CPhidget_close', handleLeft);
calllib('phidget21', 'CPhidget_delete', handleLeft);
calllib('phidget21', 'CPhidget_close', handleRight);
calllib('phidget21', 'CPhidget_delete', handleRight);

sys = [];

% end mdlTerminate


