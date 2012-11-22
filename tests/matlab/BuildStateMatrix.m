function [S] = BuildStateMatrix(dsys_ar_a, dsys_ar_c)
global kN;

num_states = size(dsys_ar_a(:, :, 1),2);
num_periods = size(dsys_ar_a,3);
S = zeros(kN, num_states);
state_mat_pow = eye(num_states);     %product of state matrices
for period_num = 1:num_periods
	state_mat_pow = state_mat_pow * dsys_ar_a(:,:,period_num);
    % C*Ad
    S(period_num,:) = dsys_ar_c(:,:,period_num) * state_mat_pow;
end