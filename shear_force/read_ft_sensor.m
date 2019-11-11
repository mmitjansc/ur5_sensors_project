%% Read .csv file from finger sensors

close all

M = readmatrix('ft_sensor.csv');

for i=3:5
    hold on
    plot((M(:,1) - M(1,1)) + M(:,2).*1e-9,M(:,i));
end

title('Force values from FT sensor');
xlabel('Time (s)');
ylabel('Force (N)');
grid on