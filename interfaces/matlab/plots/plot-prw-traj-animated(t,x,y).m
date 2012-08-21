%% Previewed
nbsamples = 16;
di = 10;
for i = 1:di:length(com_prw.time) - di
    subplot(1,2,1);
    lines_real_x =  plot(com.time(i,1), com.signals.values(i, 1),'xb');
    lines_prw_x = plot(com_prw.signals.values(i, 1:nbsamples), com_prw.signals.values(i, nbsamples+1:2*nbsamples),'red');
    hold on;
    subplot(1,2,2);
    lines_real_y = plot(com.time(i,1), com.signals.values(i, 2),'black');
    lines_prw_y = plot(com_prw.signals.values(i, 1:nbsamples), com_prw.signals.values(i, 2*nbsamples+1:3*nbsamples),'red');
    hold on;
    pause;
end

%% Legend
subplot(1,2,1);
legend([lines_prw_x, lines_real_x],'com_{prw}^x','com_{real}^x');
xlabel('Time [s]')
ylabel('X [m]')
title('CoM Positions (previewed and realized)');
subplot(1,2,2);
legend([lines_prw_y, lines_real_y],'com_{prw}^y','com_{real}^y');
xlabel('Time [s]')
ylabel('Y [m]')
title('CoM Positions (previewed and realized)');
