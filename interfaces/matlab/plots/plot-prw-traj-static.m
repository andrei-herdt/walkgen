figure;
%% Previewed
nbsamples = 16;
di = 1;
for i = 1:di:length(com_prw.time) - di
    subplot(1,2,1);
    lines_prw_com_x = plot(com_prw.signals.values(i, 1:nbsamples), com_prw.signals.values(i, nbsamples+1:2*nbsamples),'red');
    lines_prw_cop_x = plot(cop_prw.signals.values(i, 1:nbsamples), cop_prw.signals.values(i, nbsamples+1:2*nbsamples),'black');
    lines_prw_cp_x = plot(cp_prw.signals.values(i, 1:nbsamples), cp_prw.signals.values(i, nbsamples+1:2*nbsamples),'magenta');
    %lines_com_control_x = plot(com_control.signals.values(i, 1:nbsamples), com_control.signals.values(i, nbsamples+1:2*nbsamples),'cyan');
    hold on;
    subplot(1,2,2);
    lines_prw_com_y = plot(com_prw.signals.values(i, 1:nbsamples), com_prw.signals.values(i, 2*nbsamples+1:3*nbsamples),'red');
    lines_prw_cop_y = plot(cop_prw.signals.values(i, 1:nbsamples), cop_prw.signals.values(i, 2*nbsamples+1:3*nbsamples),'black');
    lines_prw_cp_y = plot(cp_prw.signals.values(i, 1:nbsamples), cp_prw.signals.values(i, 2*nbsamples+1:3*nbsamples),'magenta');
    %lines_com_control_y = plot(com_control.signals.values(i, 1:nbsamples), com_control.signals.values(i, 2*nbsamples+1:3*nbsamples),'cyan');
    hold on;
end

%% Realized
subplot(1,2,1);
lines_real_com_x = plot(com.time(1:di:length(com.time),1), com.signals.values(1:di:length(com.time), 1),'green');
lines_real_cop_x = plot(cop.time(1:di:length(com.time),1), cop.signals.values(1:di:length(com.time), 1),'blue');
subplot(1,2,2);
lines_real_com_y = plot(com.time(1:di:length(com.time),1), com.signals.values(1:di:length(com.time), 2),'green');
lines_real_cop_y = plot(cop.time(1:di:length(com.time),1), cop.signals.values(1:di:length(com.time), 2),'blue');

%% Support states
subplot(1,2,1);
ylim=get(gca,'ylim');
lines_support_x = line([support.signals.values(:,1)';support.signals.values(:,1)'],ylim.', 'LineStyle',':', 'LineWidth',1,'Color',[.5 .5 .5]);%
subplot(1,2,2);
ylim=get(gca,'ylim');
lines_support_y = line([support.signals.values(:,1)';support.signals.values(:,1)'],ylim.', 'LineStyle',':', 'LineWidth',1,'Color',[.5 .5 .5]);%[.8 .8 .8]

%% Legend
subplot(1,2,1);
legend([lines_prw_com_x, lines_prw_cop_x, lines_prw_cp_x, lines_real_com_x, lines_real_cop_x, lines_support_x(1)], 'com_{prw}^x','cop_{prw}^x','cp_{prw}^x','com_{real}^x','cop_{real}^x','new support');
xlabel('Time [s]');
ylabel('X [m]');
title('CoM Positions (previewed and realized)');
subplot(1,2,2);
legend([lines_prw_com_y, lines_prw_cop_y, lines_prw_cp_y, lines_real_com_y, lines_real_cop_y, lines_support_y(1)], 'com_{prw}^y','cop_{prw}^y','cp_{prw}^y','com_{real}^y','cop_{real}^y','new support');
xlabel('Time [s]');
ylabel('Y [m]');
title('CoM Positions (previewed and realized)');

%% Clear
clear lines_prw_com_x lines_prw_cop_x lines_prw_cp_x lines_prw_com_y lines_prw_cop_y lines_real_com_x lines_real_cop_x lines_real_com_y lines_real_cop_y lines_com_control_x lines_com_control_y