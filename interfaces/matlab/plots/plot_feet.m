figure
%% Trajectories
subplot(1,2,1);
lines_left_foot_x = plot(left_foot.time, left_foot.signals.values(:,1),'green');
hold on;
lines_right_foot_x = plot(right_foot.time, right_foot.signals.values(:,1),'blue');
hold on;
lines_next_foot_x = plot(next_foot.time, next_foot.signals.values(:,1),'--rs','LineWidth',2, 'MarkerEdgeColor','k', 'MarkerFaceColor','g', 'MarkerSize',2);
hold on;
lines_real_cop_x = plot(cop.time(1:length(cop.time),1), cop.signals.values(1:length(cop.time), 1),'black');

subplot(1,2,2);
lines_left_foot_y = plot(left_foot.time, left_foot.signals.values(:,2),'green');
hold on;
lines_right_foot_y = plot(right_foot.time, right_foot.signals.values(:,2),'blue');
hold on;
lines_next_foot_y = plot(next_foot.time, next_foot.signals.values(:,2),'--rs','LineWidth',2, 'MarkerEdgeColor','k', 'MarkerFaceColor','k', 'MarkerSize',2);
hold on;
lines_real_cop_y = plot(cop.time(1:length(cop.time),1), cop.signals.values(1:length(cop.time), 2),'black');
            
%% Support states
subplot(1,2,1);
ylim=get(gca,'ylim');
lines_support_x = line([support.signals.values(:,1)';support.signals.values(:,1)'],ylim.', 'LineStyle',':', 'LineWidth',1,'Color',[.5 .5 .5]);%
subplot(1,2,2);
ylim=get(gca,'ylim');
lines_support_y = line([support.signals.values(:,1)';support.signals.values(:,1)'],ylim.', 'LineStyle',':', 'LineWidth',1,'Color',[.5 .5 .5]);%[.8 .8 .8]

%% Graph description
subplot(1,2,1)
legend([lines_left_foot_x, lines_right_foot_x, lines_next_foot_x, lines_real_cop_x, lines_support_x(1)], 'foot_{left}^x','foot_{right}^x','foot_{next}^x','cop_{real}^x','new support');
xlabel('Time [s]');
ylabel('X [m]');
title('Sagittal foot positions (previewed and realized)');

subplot(1,2,2)
legend([lines_left_foot_y, lines_right_foot_y, lines_next_foot_y, lines_real_cop_y, lines_support_y(1)], 'foot_{left}^y','foot_{right}^y','foot_{next}^y','cop_{real}^y','new support');
xlabel('Time [s]')
ylabel('Y [m]')
title('Frontal foot positions (previewed and realized)')