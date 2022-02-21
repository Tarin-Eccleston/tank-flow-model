% calculate 3% settling times for tanks 1 & 2

function[Ts_1,Ts_2,pumps_std_1,pumps_std_2] = calc_settle(tout,heights,pumps)

h1_ref = 200; %disp(['h1_ref = ',num2str(h1_ref)]);
h2_ref = 150; %disp(['h2_ref = ',num2str(h2_ref)]);

[b,a]=butter(5, 0.1);                   % create 5th-order low-pass Butterworth filter with normalised cut-off freq of 0.1
heights_filt = filtfilt(b,a,heights);   % filter the height signals both ways (zero delay)

dh1 = abs(heights_filt(:,1) - h1_ref);  % absolute errors
dh2 = abs(heights_filt(:,2) - h2_ref);

i = length(dh1);
while dh1(i) < 0.03*h1_ref
    i = i - 1;
end

Ts_1 = round(tout(i+1));
%disp(['Settling time for tank 1 = ',num2str(Ts_1),' sec']);

i = length(dh2);
while dh2(i) < 0.03*h2_ref
    i = i - 1;
end

Ts_2 = round(tout(i+1));
%disp(['Settling time for tank 2 = ',num2str(Ts_2),' sec']);

% calculate the standard deviation of steady-state pump flow rates
pumps_std_1 = 0; 
pumps_std_2 = 0; 
if max([Ts_1, Ts_2]) < tout(end)-50      % only perform calc if system has settled for at least 50 seconds before end of sim
    tstart = find(tout == tout(end)-20); % sample number 20 seconds from end
    pumps_std = std(pumps(tstart:end,:));
    %disp(['Standard deviation of pump 1 = ',num2str(pumps_std(1)),' mm/s']);
    %disp(['Standard deviation of pump 2 = ',num2str(pumps_std(2)),' mm/s']);
    pumps_std_1 = pumps_std(1); 
    pumps_std_2 = pumps_std(2); 
end
%disp(' ');    



end

