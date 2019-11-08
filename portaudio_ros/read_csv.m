%% Read .csv file from finger sensors

close all

M = readmatrix('ys.csv');

Ts = 1/44100;

t = 0:Ts:(length(M)-1)*Ts;

plot(t,M);
xlabel('Time (s)');
ylabel('Signal strength');
title('PortAudio output');