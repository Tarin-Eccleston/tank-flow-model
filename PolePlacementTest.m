%% Setup

G1o = 0;  %Outflow tank 1
G2o = 0;  %Outflow tank 2
G3o = 1;  %Outflow tank 3
G12 = 1;  %Crossflow tank 1 to tank 2
G32 = 1;  %Crossflow tank 3 to tank 2
a0 = 0.461358435228121;
b0 = 2.036156237776855;
ac = 0.610956516241893;
bc = 0; 
H1 = [-33.648281245793970,3.333521763708310e+02];   
H2 = [-33.765532144786600,3.358927606699081e+02];
H3 = [-33.660513524794340,3.290456571205451e+02];   
P1 = [9.591077891437601,0.959107789143760];      
P2 = [8.738131401544525,0.873813140154452]; 
Qlimit = 16;       % [mm/s]
Hlimit = 350;      % [mm]

load('LinearModel_C');
A_bar = [zeros(2),C;zeros(3,2),A]; 
B_bar = [zeros(2);B];

% Test controllability
Mc_bar = ctrb(A_bar,B_bar);
rnk_bar = rank(Mc_bar);
disp('Rank of Mc_bar is:');
disp(rnk_bar); 
disp('Rank is 5, therefore reference tracking system is controllable with only force input');

%% Running simulations
p1_range = 0.02:0.005:0.05;
p2_range = 0.06:0.02:0.3;
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

%% Plotting

figure('Name','Settling Time With Different Pole Placements')
title('Settling Time (H1)');
xlabel('Pole position - real');
ylabel('Settling time (s)');
hold on; grid on;
for i = 1:length(p1_range)
    plot(p2_range, Ts_1_tuning((1:length(p2_range)) + (i-1)*length(p2_range)), 'DisplayName', ['Dominant pole at -', num2str(p1_range(i))]);
end
ylim([0 max(Ts_1_tuning)])
legend('show')

figure('Name','Settling Time With Different Pole Placements')
title('Settling Time (H2)');
xlabel('Non-dominant pole position - real');
ylabel('Settling time (s)');
hold on; grid on;
for i = 1:length(p1_range)
    plot(p2_range, Ts_2_tuning((1:length(p2_range)) + (i-1)*length(p2_range)), 'DisplayName', ['Dominant pole at -', num2str(p1_range(i))]);
end
ylim([0 max(Ts_2_tuning)])
legend('show')

figure('Name','Standard Deviation With Different Pole Placements')
title('Standard Deviation (U1)');
xlabel('Non-dominant pole position - real');
ylabel('Deviation in flow rate (mm/s)');
hold on; grid on;
for i = 1:length(p1_range)
    plot(p2_range, Sd_1_tuning((1:length(p2_range)) + (i-1)*length(p2_range)), 'DisplayName', ['Dominant pole at -', num2str(p1_range(i))]);
end
ylim([0 max(Sd_1_tuning)])
legend('show')

figure('Name','Standard Deviation With Different Pole Placements')
title('Standard Deviation (U2)');
xlabel('Non-dominant pole position - real');
ylabel('Deviation in flow rate (mm/s)');
hold on; grid on;
for i = 1:length(p1_range)
    plot(p2_range, Sd_2_tuning((1:length(p2_range)) + (i-1)*length(p2_range)), 'DisplayName', ['Dominant pole at -', num2str(p1_range(i))]);
end
ylim([0 max(Sd_2_tuning)])
legend('show')


