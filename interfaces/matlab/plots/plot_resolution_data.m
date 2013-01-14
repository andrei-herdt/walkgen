figure;
%% Plot data
subplot(3,1,1);
lines_res_time = plot(resolution_data.time, resolution_data.signals.values(:,4),'blue');
ylim = get(gca,'ylim');
lines_support_1 = line([support.signals.values(:,1)';support.signals.values(:,1)'],ylim.', 'LineStyle',':', 'LineWidth',1,'Color',[.5 .5 .5]);%
subplot(3,1,2);               
lines_ws_recomput = plot(resolution_data.time, resolution_data.signals.values(:,2),'blue');
ylim=get(gca,'ylim');
lines_support_2 = line([support.signals.values(:,1)';support.signals.values(:,1)'],ylim.', 'LineStyle',':', 'LineWidth',1,'Color',[.5 .5 .5]);%
subplot(3,1,3);
lines_obj_value = plot(resolution_data.time, resolution_data.signals.values(:,3),'red');
%% Graph description
subplot(3,1,1);
legend([lines_res_time, lines_support_1(1)], 'resolution time', 'support change');
ylabel('Time [mus]');
title('Resolution data');
subplot(3,1,2);
legend([lines_ws_recomput, lines_support_2(1)], 'num. workset rec.', 'support change');
ylabel('Num. iterations ');
subplot(3,1,3);
legend(lines_obj_value, {'objective value'});
xlabel('Time [s]');

%% Clean
clear lines_res_time lines_ws_recomput lines_support_1 lines_support_2 com_lines cop_lines left_foot_lines right_foot_lines
