%{
%%%%%%%%%%%%%%% Track Vehicle Parameter %%%%%%%%%%%%%%%%%%%%%%%%%
VmaxDR = 0.9*0.5350; 
rNominalDR = 0.052959; % [m] Nominal Wheel Radius in GNC
bDR = 0.9*(92/70)*.31;%0.5842 ; % [m] Effective Platform Width 

%%%%%%%%%%%%%%% Encoder Parameter %%%%%%%%%%%%%%%%%%%%%%%%%
eTickDR = 0.9*237;%236.8852;%900; % 866.1417-905.5118; % [ticks/m] number of ticks per 1 m of vehicle translation % from 22-23 [ticks/inch]
TsampleEncoderDR = 1/10; %1/100; % 0.1 [s] Encoder sample time in GNC processing
TauEncoderDR = 0.1; %.5; %10;%.5;

%%%%%%%%%%%%%%% Speed -> Duty Cycle - Conversion %%%%%%%%%%%%%%%%%%%%%%%%%
% dcArrayDR = [-90 -10 0 10 90];
% wArrayDR  = [-VmaxDR, -VmaxDR/100, 0,   VmaxDR/100, VmaxDR]./rNominalDR ;

dcArrayDR = [-100 100];
wArrayDR  = [-VmaxDR, VmaxDR]./rNominalDR ;
%}
%%

% if we want the estimation parameters identical to actual parameters
VmaxDR = Vmax; 
rNominalDR = rNominal;
bDR = b;
eTickDR = eTick;
TsampleEncoderDR = TsampleEncoder;
TauEncoderDR = 0.1;

dcArrayDR = [-100 -11 -10 0 10 11 100];
wArrayDR  = [-Vmax, -Vmin, -Vmin/100, 0,   Vmin/100, Vmin, Vmax]./rNominal;
%%

%%
%%%%%%%%%%%%%%%%%%% control parameter %%%%%%%%%%%%%%%%%%%%%%%%%%%
KP1=10; %20;    % 1 is velocity controller
KP2=3;%10;  %7 ->   % 2 is angle controller
KI1=.5;%0.3; %0.001; %0.001;%10;
KI2=0.001; %0.001; %0.001;%1; % 3 ->
KD1=0; %3; %0; %3; %0;%3;
KD2=0;% 3;%  %.5; %0; %0.5; %0;%0.5; %0.5 ->
Tsample = 1/10; %1/100; %1/10;   %sampling rate -> 0.01
Tmodel=Tsample;
Tau1 = 0.01;     %time constant of filter 1
Tau2 = 0.1;%0.1;       %time constant of filter 2
%%%%%%%%%%%%%%%%%%% guidance parameter %%%%%%%%%%%%%%%%%%%%%%%%%%%
rp1 =  0.3; % 1 -> [m] proximity circle to start slowing down
rp2 =  0.2;  % [m] radius of wayPoint proximity circle to switch to the next wayPoint
Vcom = 3*VmaxDR/4; % [m/s] used when constant speed is commanded

