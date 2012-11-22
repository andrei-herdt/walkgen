function [input_mat] = BuildInputMatrix(dsys_ar_a, dsys_ar_b, dsys_ar_c)
global kN;

num_states = size(dsys_ar_a(:, :, 1), 2);
input_mat = zeros(kN, kN);
for col = 1:kN
    state_mat_pow = eye(num_states);
    for row = col:kN
        % C*A*B
        input_mat(row, col) = dsys_ar_c(:,:,row) * state_mat_pow * dsys_ar_b(:,:,col);
        state_mat_pow = state_mat_pow * dsys_ar_a(:,:,row);
    end
end