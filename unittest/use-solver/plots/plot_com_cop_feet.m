com_lines = plot(com.signals.values(:,1), com.signals.values(:,2),'red');
hold on
cop_lines = plot(cop.signals.values(:,1), cop.signals.values(:,2),'black');
hold on
left_foot_lines = plot(left_foot.signals.values(:,1), left_foot.signals.values(:,2),'green');
hold on
right_foot_lines = plot(right_foot.signals.values(:,1), right_foot.signals.values(:,2),'green');

%% Description
xlabel('X [m]')
ylabel('Y [m]')
title('CoM-CoP-Feet')
legend([com_lines, cop_lines, left_foot_lines, right_foot_lines], {'com^{x,y}','cop^{x,y}','foot_{left}^{x,y}','foot_{right}^{x,y}'})