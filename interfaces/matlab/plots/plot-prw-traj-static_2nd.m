figure;
%% Parameters
num_samples = sim_parameters.signals.values(1,1);
di = 10;

%% Plot previewed
for i = 1:di:length(com_prw.time) - di
    subplot(1,2,1);
    lines_prw_com_x = plot(com_prw.signals.values(i, 1:num_samples), com_prw.signals.values(i, num_samples+1:2*num_samples),'red');
    lines_prw_cp_x = plot(cp_prw.signals.values(i, 1:num_samples), cp_prw.signals.values(i, num_samples+1:2*num_samples),'magenta');
    lines_com_control_x = stairs(com_control.signals.values(i, 1:num_samples), com_control.signals.values(i, num_samples+1:2*num_samples),'k');
    hold on;
    subplot(1,2,2);
    lines_prw_com_y = plot(com_prw.signals.values(i, 1:num_samples), com_prw.signals.values(i, 2*num_samples+1:3*num_samples),'red');
    lines_prw_cp_y = plot(cp_prw.signals.values(i, 1:num_samples), cp_prw.signals.values(i, 2*num_samples+1:3*num_samples),'magenta');
    lines_com_control_y = stairs(com_control.signals.values(i, 1:num_samples), com_control.signals.values(i, 2*num_samples+1:3*num_samples),'k');
    hold on;
end

%% Plot realized
subplot(1,2,1);
lines_real_com_x = plot(com.time(2:di:length(com.time),1), com.signals.values(1:di:length(com.time)-1, 1),'green');
lines_real_cop_x = plot(cop.time(2:di:length(com.time),1), cop.signals.values(1:di:length(com.time)-1, 1),'blue');
subplot(1,2,2);
lines_real_com_y = plot(com.time(2:di:length(com.time),1), com.signals.values(1:di:length(com.time)-1, 2),'green');
lines_real_cop_y = plot(cop.time(2:di:length(com.time),1), cop.signals.values(1:di:length(com.time)-1, 2),'blue');
 
%% Plot support state changes
subplot(1,2,1);
ylim=get(gca,'ylim');
lines_support_x = line([support.signals.values(:,1)';support.signals.values(:,1)'],ylim.', 'LineStyle',':', 'LineWidth',1,'Color',[.5 .5 .5]);%
subplot(1,2,2);
ylim=get(gca,'ylim');
lines_support_y = line([support.signals.values(:,1)';support.signals.values(:,1)'],ylim.', 'LineStyle',':', 'LineWidth',1,'Color',[.5 .5 .5]);%[.8 .8 .8]

%% Plot legend
subplot(1,2,1);
legend([lines_prw_com_x, lines_prw_cp_x, lines_com_control_x, lines_real_com_x, lines_real_cop_x, lines_support_x(1)], 'com_{prw}^x','cp_{prw}^x','cop_{prw}^x','com_{real}^x','cop_{real}^x','new support');
xlabel('Time [s]');
ylabel('X [m]');
title('CoM Positions (previewed and realized)');
subplot(1,2,2);
legend([lines_prw_com_y, lines_prw_cp_y, lines_com_control_y, lines_real_com_y, lines_real_cop_y, lines_support_y(1)], 'com_{prw}^y','cp_{prw}^y','cop_{prw}^y','com_{real}^y','cop_{real}^y','new support');
xlabel('Time [s]');
ylabel('Y [m]');
title('CoM Positions (previewed and realized)');

%% Clear variables
clear lines_prw_com_x lines_prw_cp_x lines_prw_com_y lines_real_com_x lines_real_cop_x lines_real_com_y lines_real_cop_y lines_com_control_x lines_com_control_y