%% Continuous 2-order system to drive the voice coil

%%%%% User inputs %%%%%%%%%%%%%%%%%%
st = 1; % Settling time (s)
f = 100; % Frequency of signal (Hz)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
format long

syms x y z
s = tf('s');

wd = f*2*pi;

sol = solve(st==4/(x*y), wd == y*sqrt(1-x^2));
e = double(sol.x);
wn = double(sol.y); 
b = sqrt(1-e^2);

% peak time for impulse response is half of the one for step response:
tp = pi/(2*wn*b);

% So that max voltage == 1
eq = wn/b*exp(-e*wn*tp)*sin(wn*b*tp)*z;
k = double(solve(eq == 1));

G = wn^2/(s^2 + 2*e*wn*s + wn^2)*k;

%% Transform G(s) to H(z)

Ts = 1/44100;

H = c2d(G,Ts,'zoh');

figure;
impulse(H)

[a,b,c,d] = arg2vars(H.num{1}(2), H.num{1}(3), H.den{1}(2), H.den{1}(3)); 

a
b
c
d

%% Home-made simulation

% N = length(yi);
% u = zeros(1,N);
% u(1) = 1;
% y = zeros(1,N);
% 
% outs = zeros(1,N);

% for i = 1:N
% 
%     sol = (a*u(2) + b*u(3))/Ts - c*y(2) - d*y(3);
%     
%     y(1) = sol;
%     outs(i) = sol;
%     
%     u = [0,u(1:end-1)];
%     y = [0,y(1:end-1)];
%     
% end
% 
% figure;
% plot(0:Ts:(N-1)*Ts,outs)
% xlabel('Time (s)'); ylabel('Output'); 
