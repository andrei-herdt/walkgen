figure;
%% Data
subplot(3,1,1);
lines_res_time = plot(resolution_data.time, resolution_data.signals.values(:,1),'blue');
ylim = get(gca,'ylim');
lines_support_1 = line([support.signals.values';support.signals.values'],ylim.', 'LineStyle',':', 'LineWidth',1,'Color',[.5 .5 .5]);%
subplot(3,1,2);               
lines_ws_recomput = plot(resolution_data.time, resolution_data.signals.values(:,2),'blue');
ylim=get(gca,'ylim');
lines_support_2 = line([support.signals.values';support.signals.values'],ylim.', 'LineStyle',':', 'LineWidth',1,'Color',[.5 .5 .5]);%
subplot(3,1,3);
com_lines = plot(com.signals.values(:,1), com.signals.values(:,2),'red');
hold on
cop_lines = plot(cop.signals.values(:,1), cop.signals.values(:,2),'black');
hold on
left_foot_lines = plot(left_foot.signals.values(:,1), left_foot.signals.values(:,2),'green');
hold on
right_foot_lines = plot(right_foot.signals.values(:,1), right_foot.signals.values(:,2),'green');

%% Graph description
subplot(3,1,1);
legend([lines_res_time, lines_support_1(1)], 'resolution time', 'support change');
ylabel('Time [s]');
title('Resolution data');
subplot(3,1,2);
legend([lines_ws_recomput, lines_support_2(1)], 'num. workset rec.', 'support change');
xlabel('Time [s]');
ylabel('Num. iterations ');
subplot(3,1,3);
legend([com_lines, cop_lines, left_foot_lines, right_foot_lines], {'com^{x,y}','cop^{x,y}','foot_{left}^{x,y}','foot_{right}^{x,y}'})
xlabel('X [m]');
ylabel('Y [m] ');

%% Clean
clear lines_res_time lines_ws_recomput lines_support_1 lines_support_2 com_lines cop_lines left_foot_lines right_foot_lines
