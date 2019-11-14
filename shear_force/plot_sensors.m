%% Read .csv file from finger sensors

close all

M = readmatrix('csv/ft_sensor_2.csv');
L = readmatrix('csv/fingertip_2.csv');

subplot(2,1,1);

for i=3:5
    hold on
    plot((M(:,1) - M(1,1)) + M(:,2).*1e-9, M(:,i));
end

title('Force values from FT sensor');
xlabel('Time (s)');
ylabel('Force (N)');
grid on
legend('Fx','Fy','Fz');

% Fingertip values

subplot(2,1,2);

for i=3:8
    hold on
    plot((L(:,1) - L(1,1)) + L(:,2).*1e-9, L(:,i));
end

title('Force values from fingertips');
xlabel('Time (s)');
ylabel('Pressure (u)');
grid on
legend('ft1','ft2','ft3','ft4','ft5','ft6');