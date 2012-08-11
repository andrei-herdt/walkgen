figure
subplot(1,2,1);
lines_left_foot_x = plot(left_foot.time, left_foot.signals.values(:,1),'green');
hold on;
lines_right_foot_x = plot(right_foot.time, right_foot.signals.values(:,1),'blue');
hold on;
lines_next_foot_x = plot(next_foot.time, next_foot.signals.values(:,1),'--rs','LineWidth',2,...
                'MarkerEdgeColor','k',...
                'MarkerFaceColor','g',...
                'MarkerSize',2);
hold on;
% nbsamples = 16;
% for i = 1:length(cop_prw.time)
% lines_prw_cop_x = plot(cop_prw.signals.values(i, 1:nbsamples), cop_prw.signals.values(i, nbsamples+1:2*nbsamples),'black');
% end
lines_real_cop_x = plot(cop.time(1:length(cop.time),1), cop.signals.values(1:length(cop.time), 1),'black');
legend([lines_left_foot_x, lines_right_foot_x, lines_next_foot_x, lines_real_cop_x], 'foot_{left}^x','foot_{right}^x','foot_{next}^x','cop_{real}^x');
xlabel('Time [s]');
ylabel('X [m]');
title('Sagittal foot positions (previewed and realized)');

subplot(1,2,2)
lines_left_foot_y = plot(left_foot.time, left_foot.signals.values(:,2),'green');
hold on
lines_right_foot_y = plot(right_foot.time, right_foot.signals.values(:,2),'blue');
hold on
lines_next_foot_y = plot(next_foot.time, next_foot.signals.values(:,2),'--rs','LineWidth',2,...
                'MarkerEdgeColor','k',...
                'MarkerFaceColor','k',...
                'MarkerSize',2);
hold on;
lines_real_cop_y = plot(cop.time(1:length(cop.time),1), cop.signals.values(1:length(cop.time), 2),'black');
            
%% Description            
legend([lines_left_foot_y, lines_right_foot_y, lines_next_foot_y, lines_real_cop_y], 'foot_{left}^y','foot_{right}^y','foot_{next}^y','cop_{real}^y');
xlabel('Time [s]')
ylabel('Y [m]')
title('Frontal foot positions (previewed and realized)')