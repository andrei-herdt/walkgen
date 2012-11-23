clear;

%% Global variables
global kN;

%% Constants
g = 9.81;
h = 0.814;
kT = 0.1;
kN = 16;
omega = sqrt(g/h);

%% Dynamics
%%% Continuous model
%%%% Third order model (not tested!)
Ac = [0, 1, 0; 0, 0, 1; 0, 0, 0];
Bc = [0; 0; 1];
%C = [1, 0, 0];          	%position
%C = [0, 1, 0]; 			%velocity
%C = [1, 0, -1/omega^2];    %CoP
C = [1, 1/omega, 0];        %CP
%%%% Second order model
%Ac = [0, 1; omega^2, 0];
%Bc = [0; -omega^2];
%C = [1, 0];          %position
%C = [0, 1];          %velocity
%C = [1, 1/omega];    %capture point

%Ac = [-omega, omega; 0, omega];
%Bc = [0; -omega];
%Ac = [-omega, 0; 0, omega];
%Bc = [omega; -omega];
%C = [1/2, 1/2];          %position
%C = [0, 1];              %capture point
%%%% Regularization
kRegCoeff = 10e-5;
% Minimize control
R = kRegCoeff * eye(kN);	
% Minimize derivative of control
%R = eye(kN);
%R(2:kN+1:kN*kN) = -1;
%R = kRegCoeff * R;

csys = ss(Ac, Bc, C, []);

%%%% Standard form
num_sampling_periods = 100;	% number of recomputations
cond_num_vec = zeros(1, num_sampling_periods);
reg_cond_num_vec = zeros(1, num_sampling_periods);
for num_period = 1:num_sampling_periods
	% Sampling periods
	sample_periods_vec = kT * ones(1, kN);
	sample_periods_vec(1) = kT / num_period;    %First sampling period
	% Split LTI array because Matlab does not allow for varying sampling periods in LTI arrays
	[dsys_arr_a, dsys_arr_b, dsys_arr_c] = BuildDiscreteSystems(csys, sample_periods_vec);
	S = BuildStateMatrix(dsys_arr_a, dsys_arr_c);
	U = BuildInputMatrix(dsys_arr_a, dsys_arr_b, dsys_arr_c);
	hessian = U' * U;
	cond_num_vec(num_period) = cond(hessian);
	hessian_reg = hessian + R;
	reg_cond_num_vec(num_period) = cond(hessian_reg);
end
disp('Condition number(s) (standard form)')
disp(cond_num_vec);
disp('Condition number(s) (standard form - regularized)')
disp(reg_cond_num_vec);
plot(cond_num_vec);
hold;
plot(reg_cond_num_vec);

%%%% Decoupled form (Goodwin - Chapter11) with constant sampling rate
% We require a diagonal state matrix
Ac = [-omega, 0; 0, omega];    
Bc = [omega; -omega];
C = [1/2, 1/2];          %position
%C = [-omega/2, omega/2]; %velocity
%C = [0, 1];              %capture point

dsys = c2d(csys, kT);

As = dsys.A(1,1);
Au = dsys.A(2,2);
Ad_vec = [];
for i = 1:kN
   Ad_vec = [Ad_vec; [As^i, 0; 0, Au^(-i)]]; %powers of A
end

S_s = zeros(kN, 1);      %
S_u = zeros(kN, 1);      %
U_s = zeros(kN, kN);     %
U_u = zeros(kN, kN);     %
U_s(1:kN+1:end) = dsys.C(1) * dsys.B(1);
for i = 1:kN-1
    CABs = dsys.C(1) * Ad_vec(2*i-1, 1) * dsys.B(1);
    CABu = -dsys.C(2) * Ad_vec(2*i, 2) * dsys.B(2);
    %fill diagonal i+1
    for j = 1:kN-i
        U_s(i+j, j) = CABs;
        U_u(j, i+j) = CABu;
    end
end
U = U_s + U_u;
Hdec = U'*U;
disp('Condition number of regularized sub hessian (decoupled modes): ');
disp(cond(Hdec));
