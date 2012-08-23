figure;
%% Previewed
nbsamples = 16;
di = 10;
for i = 1:di:length(com_prw.time) - di
    subplot(1,2,1);
    lines_prw_com_x = plot(com_prw.signals.values(i, 1:nbsamples), com_prw.signals.values(i, nbsamples+1:2*nbsamples),'red');
    lines_prw_cop_x = plot(cop_prw.signals.values(i, 1:nbsamples), cop_prw.signals.values(i, nbsamples+1:2*nbsamples),'black');
    hold on;
    subplot(1,2,2);
    lines_prw_com_y = plot(com_prw.signals.values(i, 1:nbsamples), com_prw.signals.values(i, 2*nbsamples+1:3*nbsamples),'red');
    lines_prw_cop_y = plot(cop_prw.signals.values(i, 1:nbsamples), cop_prw.signals.values(i, 2*nbsamples+1:3*nbsamples),'black');
    hold on;
end

%% Realized
subplot(1,2,1);
lines_real_com_x = plot(com.time(1:di:length(com.time),1), com.signals.values(1:di:length(com.time), 1),'green');
lines_real_cop_x = plot(cop.time(1:di:length(com.time),1), cop.signals.values(1:di:length(com.time), 1),'blue');
subplot(1,2,2);
lines_real_com_y = plot(com.time(1:di:length(com.time),1), com.signals.values(1:di:length(com.time), 2),'green');
lines_real_cop_y = plot(cop.time(1:di:length(com.time),1), cop.signals.values(1:di:length(com.time), 2),'blue');

%% Legend
subplot(1,2,1);
legend([lines_prw_com_x, lines_prw_cop_x, lines_real_com_x, lines_real_cop_x], 'com_{prw}^x','cop_{prw}^x','com_{real}^x','cop_{real}^x');
xlabel('Time [s]');
ylabel('X [m]');
title('CoM Positions (previewed and realized)');
subplot(1,2,2);
legend([lines_prw_com_y, lines_prw_cop_y, lines_real_com_y, lines_real_cop_y], 'com_{prw}^y','cop_{prw}^y','com_{real}^y','cop_{real}^y');
xlabel('Time [s]');
ylabel('Y [m]');
title('CoM Positions (previewed and realized)');

%% Clear
clear lines_prw_com_x lines_prw_cop_x lines_prw_com_y lines_prw_cop_y lines_real_com_x lines_real_cop_x lines_real_com_y lines_real_cop_y