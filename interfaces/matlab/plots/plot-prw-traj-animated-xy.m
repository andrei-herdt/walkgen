%% Previewed
num_samples = sim_parameters.signals.values(1,1);
di = 10;
ndi = 10;
for i = 1:di*ndi:length(com_prw.time) - di*ndi
    for j = i:di:i+di*ndi
        lines_real =  plot(com.signals.values(j, 1), com.signals.values(j, 2), 'xb');
        lines_prw_x = plot(com_prw.signals.values(j, num_samples+1:2*num_samples), com_prw.signals.values(j, 2*num_samples+1:3*num_samples),'red');
        hold on;
    end
    pause;
end

%% Legend
legend([lines_prw, lines_real],'com_{prw}^{x,y}','com_{real}^{x,y}');
xlabel('X [m]')
ylabel('Y [m]')
title('CoM Positions (previewed and realized)');