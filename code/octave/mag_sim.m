disp("Magnetometer Simulation")

close all
clear all
#function [centerX, centerY] = circ_middle(x1,y1,x2,y2,x3,y3)
#  yDelta_a = y2-y1
#  xDelta_a = x2-x1
#  yDelta_b = y3-y2
#  xDelta_b = x3-x2
#  centerX = 0
#  centerY = 0
#  aSlope = yDelta_a/xDelta_a
#  bSlope = yDelta_b/yDelta_b
#  centerX = (aSlope*bSlope*(y1 - y3) + bSlope*(x1+x2) - aSlope*(x2 + x3) ) ...
#                   / (2*(bSlope - aSlope))
#  centerY = -1*( centerX - (x1+x2)/2 )/aSlope + (y1+y2)/2
#end

function [centerX, centerY,r] = circ_middle(x1,y1,x2,y2,x3,y3)
  A = x1*(y2-y3) - y1*(x2-x3) + x2*y3 - x3*y2;
  sq1 = (x1*x1 + y1*y1);
  sq2 = (x2*x2 + y2*y2);
  sq3 = (x3*x3 + y3*y3);
  B = sq1*(y3-y2) + sq2*(y1-y3) + sq3*(y2-y1);
  C = sq1*(x2-x3) + sq2*(x3-x1) + sq3*(x1-x2);
  D = sq1*(x3*y2-x2*y3) + sq2*(x1*y3-x3*y1) + sq3*(x2*y1-x1*y2);
  centerX = -1*B/(2*A);
  centerY = -1*C/(2*A);
  r = sqrt((B*B + C*C - 4*A*D)/(4*A*A));
end



#[x,y] = circ_middle2(2*cos(1),2*sin(1),0,2,-2,0)
#[x,y] = circ_middle2(1,1,2,4,5,3) # s/b (3,2)


h = [ 17.739; 0.004; 51724];
hHorz = h(1:2,:)
hMagTrue = sqrt(hHorz'*hHorz);
noise = 0.5;

B = 10*randn(2,1)
S = 1+0.1*randn(2,1)

 
samples = [0:5:360*4];
samples = mod(samples,360);
nSamples = length(samples);
samplesRad = deg2rad(samples);
cMatrix = cos(samplesRad);
sMatrix = sin(samplesRad);
hTrue = zeros(2,nSamples);
hMeas = zeros(2,nSamples);
measNoise = noise*randn(2,nSamples);
 
for s = 1:length(samples),
   att = [ cMatrix(s) -1*sMatrix(s); sMatrix(s) cMatrix(s) ];
   hTrue(:,s) = att*hHorz;
   hMeas(:,s) = diag(S)*(hTrue(:,s)+B+measNoise(:,s));
end

plot(hTrue(1,:),hTrue(2,:))
hold on
plot(hMeas(1,:),hMeas(2,:))

for s = 1:length(samples)-3,
   [cx(s),cy(s),r(s)] = circ_middle(hMeas(1,s),hMeas(2,s),hMeas(1,s+1),hMeas(2,s+1),hMeas(1,s+2),hMeas(2,s+2));
   angTrue(s) = rad2deg(atan2(hTrue(2,s+2),hTrue(1,s+2)));
   angMeas(s) = rad2deg(atan2(hMeas(2,s+2),hMeas(1,s+2)));
   cxEst = mean(cx);
   cyEst = mean(cy);
   angCalRad = atan2(hMeas(2,s+2)-cyEst,hMeas(1,s+2)-cxEst);
   angCal(s) = rad2deg(angCalRad);
   rx(s) = r(s)*cos(angCalRad);
   ry(s) = r(s)*sin(angCalRad);
   rxTrue(s) = hMagTrue*cos(angCalRad);
   ryTrue(s) = hMagTrue*sin(angCalRad);
   sfX(s) = rxTrue/rx;
   sfY(s) = ryTrue/ry;
   sfXest(s) = mean(sfX);
   sfYest(s) = mean(sfY);
   angCalRad = atan2(sfYest(s)*(hMeas(2,s+2)-cyEst),sfXest(s)*(hMeas(1,s+2)-cxEst));
   #angCal(s) = rad2deg(angCalRad);
   
   error(s) = angCal(s) - angTrue(s);
   #[cx(s),cy(s),r(s)] = circ_middle(hTrue(1,s),hTrue(2,s),hTrue(1,s+1),hTrue(2,s+1),hTrue(1,s+2),hTrue(2,s+2));
end

#error = angCal-samples(3:end-1);
idx = error > 180;
error(idx) = error(idx)-360;
idx = error < -180;
error(idx) = error(idx)+360;

figure
plot(samples(1:end-3),cx,samples(1:end-3),cy,samples(1:end-3),r)
figure
plot(samples(1:end-3),angMeas,samples(1:end-3),angCal,samples(1:end-3),angTrue)
figure
plot(error)

 
 