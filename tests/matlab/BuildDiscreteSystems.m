function [dsys_arr_a, dsys_arr_b, dsys_arr_c] = BuildDiscreteSystems(csys, sample_periods_vec)

%% Build LTI array
for period_num = 1:size(sample_periods_vec, 2)
    % Discretize
    [dsys_arr_a(:,:,period_num),dsys_arr_b(:,:,period_num), dsys_arr_c(:,:,period_num)] ...
    = Discretize(csys, sample_periods_vec(period_num));
end