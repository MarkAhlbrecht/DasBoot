% Boat Model Testing
clear all
close all

dt = 0.1;
time = 0:dt:60*10;
Km = 36;
Kw = 0.12;
Ktr = 0.044;

trMax = 5; # deg/sec
trHdgMax = 45;
trDeadZone = 1;
trHdgCoef = -1*trMax/trHdgMax;



nSamples = size(time,2);
motor = zeros(1,nSamples);
##idx = time>=10 & time<11; 
##motor(idx) = 1;
##idx = time>=20 & time<21; 
##motor(idx) = -1;
##
##idx = time>=30 & time<32.5; 
##motor(idx) = -1;
##idx = time>=40 & time<42.5; 
##motor(idx) = 1;

speed = 4.5*ones(1,nSamples);

wAngle = zeros(1,nSamples);
rAngle = zeros(1,nSamples);
tRate = zeros(1,nSamples);
hdg = zeros(1,nSamples);
hdgNoise = 0.5*randn(1,nSamples);

Khdg_PID = -0.1;
setHdg = zeros(1,nSamples);
err = zeros(1,nSamples);
ctlOutput = zeros(1,nSamples);
setHdg(10/dt:end) = 90;
setHdg(100/dt:end) = 180;
setHdg(200/dt:end) = 0;
trTarget = zeros(1,nSamples);
%Ktr_PID = -0.22;
Ktr_PID = -0.22;
measDelay = 0;
sampleDelay = measDelay*dt+1;

for s=2:nSamples-100
  % Boat Model
  %%%%%%%%%%%%%
  % Wheel
  delta_wAngle = Km*motor(s)*dt;
  wAngle(s) = wAngle(s-1) + delta_wAngle;
  
  % Rudder
  rAngle(s) = Kw*wAngle(s);
  
  % Turn Rate
  tRate(s) = Ktr * rAngle(s) * speed(s);
  
  % Heading
  delta_hdg = tRate(s)*dt;
  hdg(s) = hdg(s-1) + delta_hdg;
  
  % Controller
  %%%%%%%%%%%%%%
  if s > sampleDelay
    err(s) = hdg(s-sampleDelay)+hdgNoise(s) - setHdg(s);
    trTarget(s) = err(s)*trHdgCoef;  
    if trTarget(s) > trMax
      trTarget(s) = trMax;
    endif
    if trTarget(s) < -trMax
      trTarget(s) = -trMax;
    endif
    trErr(s) = tRate(s-sampleDelay) - trTarget(s);
    %ctlOutput(s) = Khdg_PID*err(s);
    ctlOutput(s) = Ktr_PID*trErr(s);
    if abs(err(s)) > trDeadZone
      motorSets = round(ctlOutput(s)/dt);
      direction = sign(motorSets);
      motor(s+1:s+1+abs(motorSets)) = direction;
    endif
  endif
  
endfor




plot(time,wAngle,time,rAngle)
title("Wheel Rudder Angle")
figure
plot(time,tRate,time,hdg)
title("Turn Rate - Heading")
figure
plot(time,err,time,ctlOutput)
title("Error - CTL Output")
figure
plot(time,motor)
title("Motor")
