close all
clc

%% Define object limits:

total_lim = 3;
min_lims = [0,0];
max_lims = [2,1];
rectangle('Position',[min_lims max_lims-min_lims],'EdgeColor','r','LineWidth',2)
xlim([-total_lim,total_lim])
ylim([-total_lim,total_lim])

x = -total_lim:0.1:total_lim;
y = x;
[u,v] = meshgrid(x,y);