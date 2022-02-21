%% Initialise_ MiniThreeTank_C.m    (SYSTEM C)
clear;clc;

G1o = 0;  %Outflow tank 1 (closed)
G2o = 0;  %Outflow tank 2 (closed)
G3o = 1;  %Outflow tank 3```````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````
G12 = 1;  %Crossflow tank 1 to tank 2
G32 = 1;  %Crossflow tank 3 to tank 2
% Measured valve parameters (for simulation only)
% Outflow valve parameters; outflow [mm/s] = a0*sqrt(height) + b0
a0 = 0.461358435228121;
b0 = 2.036156237776855;
% Crossflow valve parameters; crossflow [mm/s] = ac*sqrt(delta_height) + bc
ac = 0.610956516241893;
bc = 0; 
% System callibration parameters (for RT testing only)
% Water level parameters; level [mm] = voltage * H1(1) + H1(2)
H1 = [-33.648281245793970,3.333521763708310e+02];   
H2 = [-33.765532144786600,3.358927606699081e+02];
H3 = [-33.660513524794340,3.290456571205451e+02];   
%  Pump flowrate paramters; voltage = (flow [mm/s] - P1(1))/P1(2)
P1 = [9.591077891437601,0.959107789143760];      
P2 = [8.738131401544525,0.873813140154452];   
% Common pump flow rate saturation limit and maximum water level
Qlimit = 16;       % [mm/s]
Hlimit = 350;      % [mm]

load('LinearModel_C');

%% 

A_bar = [zeros(2),C;zeros(3,2),A]; 
B_bar = [zeros(2);B];

% Test controllability
Mc_bar = ctrb(A_bar,B_bar);
rnk_bar = rank(Mc_bar);
disp('Rank of Mc_bar is:');
disp(rnk_bar); 
disp('Rank is 5, therefore reference tracking system is controllable with only force input');

K_bar = place(A_bar, B_bar, [-0.05, -0.1, -0.1, -0.12, -0.12]);

Kr = K_bar(:,1:2);
Kx = K_bar(:,3:5);

% Run simulation and collect data
sim('ThreeTankSIM');
[Ts_1,Ts_2,pumps_std_1,pumps_std_2] = calc_settle(tout,y_out,pumps); 

%% Plotting

% Plotting heights
figure('name','Heights')
title('Heights')
ylabel('Height (mm)'); xlabel('Time (s)');
grid on; hold on;
plot(tout, y_out(:,1))
plot(tout, y_out(:,2))
legend({'h_1', 'h_2'})

% Plotting flow rates
figure('name','Pump Flow Rates')
title('Pump Flow Rates')
ylabel('Flow Rate (mm/s)'); xlabel('Time (s)');
grid on; hold on;
plot(tout, pumps(:,1))
plot(tout, pumps(:,2))
legend({'u_1', 'u_2'})

fprintf('Settling time 1: %i\nSettling time 2: %i\n', Ts_1, Ts_2);
fprintf('Standard deviation 1: %i\nStandard deviation 2: %i\n', pumps_std_1, pumps_std_2);
