% Track Filter Test
clear all
close all
d = csvread('D:\\Projects\\DasBoot\\data\\boat_record.csv');

time = d(:,2);
hdg = d(:,12);
track = d(:,22);
dHdg =  [0; diff(hdg)];
idx = dHdg > 180;
dHdg(idx) = dHdg(idx)-360;
idx = dHdg < -180;
dHdg(idx) = dHdg(idx)+360;
nSamples = size(track,1);
speed = d(:,21);

prevTrack = track(1);
tSamples = [1];
filtTrack = track(1);
tau = 0.75

for s=2:nSamples
  filtTrack(s) = filtTrack(s-1) + dHdg(s);
  if filtTrack(s) >= 360
    filtTrack(s) = filtTrack(s) - 360;
  endif
  if filtTrack(s) < 0
    filtTrack(s) = filtTrack(s) + 360;
  endif
  
  if track(s) != prevTrack
    tSamples(end+1) = s;
    deltaTrack = filtTrack(s-1) - track(s);
    trackTemp = track(s);
    if deltaTrack > 180
      trackTemp = trackTemp + 360;
    endif
    if deltaTrack < -180
      trackTemp = trackTemp - 360;
    endif
    if (speed(s) > 1.0)
      filtTrack(s) = tau*filtTrack(s-1) + (1-tau)*trackTemp;
    endif
    
  endif
  if filtTrack(s) >= 360
    filtTrack(s) = filtTrack(s) - 360;
  endif
  if filtTrack(s) < 0
    filtTrack(s) = filtTrack(s) + 360;
  endif
  
endfor

plot(track(tSamples))
hold on
plot(filtTrack,'ro-')

%trackFiltered = track+dHdg;
