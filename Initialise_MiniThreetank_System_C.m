% Initialise_ MiniThreeTank_C.m    (SYSTEM C)
clear;clc;
disp('Initialising parameters');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Valve positions  G=1 valve open G=0 valve closed
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
G1o = 0;  %Outflow tank 1
G2o = 0;  %Outflow tank 2
G3o = 1;  %Outflow tank 3
G12 = 1;  %Crossflow tank 1 to tank 2
G32 = 1;  %Crossflow tank 3 to tank 2

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Measured valve parameters (for simulation only)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Outflow valve parameters; outflow [mm/s] = a0*sqrt(height) + b0
a0 = 0.461358435228121;
b0 = 2.036156237776855;
% Crossflow valve parameters; crossflow [mm/s] = ac*sqrt(delta_height) + bc
ac = 0.610956516241893;
bc = 0; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       System callibration parameters (for RT testing only)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Water level parameters; level [mm] = voltage * H1(1) + H1(2)
H1 = [-33.648281245793970,3.333521763708310e+02];   
H2 = [-33.765532144786600,3.358927606699081e+02];
H3 = [-33.660513524794340,3.290456571205451e+02];   
%  Pump flowrate paramters; voltage = (flow [mm/s] - P1(1))/P1(2)
P1 = [9.591077891437601,0.959107789143760];      
P2 = [8.738131401544525,0.873813140154452];   

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Common pump flow rate saturation limit and maximum water level
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Qlimit = 16;       % [mm/s]
Hlimit = 350;      % [mm]

disp('Initialisation complete');

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

% K_bar = place(A_bar, B_bar, [-0.025, -0.20, -0.20, -0.15, -0.15]); 
% 32 182 0.049998 0.12543
% K_bar = place(A_bar, B_bar, [-0.03, -0.20, -0.20, -0.15, -0.15]);
% 32 143 0.05029 0.1456
% K_bar = place(A_bar, B_bar, [-0.025, -0.025, -0.20, -0.15, -0.15]);
% 153 166 0.029524 0.14986
% K_bar = place(A_bar, B_bar, [-0.03, -0.15, -0.15, -0.12, -0.12]);
% 42 150 0.038063 0.082698
% K_bar = place(A_bar, B_bar, [-0.035, -0.15, -0.15, -0.12, -0.12]);
% 42 127 0.039171 0.088283
% K_bar = place(A_bar, B_bar, [-0.04, -0.15, -0.15, -0.12, -0.12]);
% 42 113 0.039021 0.093076
K_bar = place(A_bar, B_bar, [-0.05, -0.15, -0.15, -0.12, -0.12]);
% 42 97 0.039035 0.10682

% Kr = K_bar(:,1:2);
% Kx = K_bar(:,3:5);

% Run simulation and collect data
% sim('ThreeTankSIM')
% [Ts_1,Ts_2,pumps_std_1,pumps_std_2] = calc_settle(tout,y_out,pumps); 

p1_range = 0.02:0.005:0.05;
p2_range = 0.1:0.05:0.3;
i = 1;
i_max = length(p1_range)*length(p2_range);
fprintf('Starting pole placement tuning:\n');
for p1 = p1_range
    for p2 = p2_range
       % Find gains
       K_bar = place(A_bar, B_bar, [-p1, -p2, -p2, -p2-0.05, -p2-0.05]);
       Kr = K_bar(:,1:2);
       Kx = K_bar(:,3:5);
       % Run simulation and get setting times and standard deviations
       sim('ThreeTankSIM')
       [Ts_1,Ts_2,pumps_std_1,pumps_std_2] = calc_settle(tout,y_out,pumps);
       Ts_1_tuning(i) = Ts_1;
       Ts_2_tuning(i) = Ts_2;
       Sd_1_tuning(i) = pumps_std_1;
       Sd_2_tuning(i) = pumps_std_2;
       fprintf('%.1f%% complete...\n', 100*i/i_max);
       i = i + 1;
    end
end
figure('Name','Settling Time')
title('Settling Time');
xlabel('Pole position - real');
ylabel('Settling time (s)');
hold on; grid on;
for i = 1:length(p1_range)
    plot(p2_range, Ts_1_tuning((1:length(p2_range)) + (i-1)*length(p2_range)), '--o');
end
figure('Name','Settling Time')
title('Settling Time');
xlabel('Pole position - real');
ylabel('Settling time (s)');
hold on; grid on;
for i = 1:length(p1_range)
    plot(p2_range, Ts_2_tuning((1:length(p2_range)) + (i-1)*length(p2_range)), '--o');
end

% % Plotting heights
% figure('name','Heights')
% title('Heights')
% ylabel('Height (mm)'); xlabel('Time (s)');
% grid on; hold on;
% plot(tout, y_out(:,1))
% plot(tout, y_out(:,2))
% legend({'h_1', 'h_2'})
% 
% % Plotting flow rates
% figure('name','Pump Flow Rates')
% title('Pump Flow Rates')
% ylabel('Flow Rate (mm/s)'); xlabel('Time (s)');
% grid on; hold on;
% plot(tout, pumps(:,1))
% plot(tout, pumps(:,2))
% legend({'u_1', 'u_2'})

