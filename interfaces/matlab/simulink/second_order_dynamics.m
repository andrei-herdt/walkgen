clear;
%%Constants:
kGravity = 9.81;
height = 0.91;
T = 0.1;
sp_first = 0.1;
sp_rest = 0.1;

cont_state_mat(1,2) = 1.;
cont_state_mat(2,1) = 1.;

%%Eigen values of A
eig_1 = -1.;
eig_2 = 1.;
eig_mat_1(1,1) = eig_1 * sp_first;
eig_mat_1(2,2) = eig_2 * sp_first;


cont_input_mat(2,1) = -height / kGravity;

[V,S] = eig(cont_state_mat);

invV = inv(V);

Ad_custom = V*diag(exp(diag(S)*T))*invV;

%%Discretization with Matlab
sys = ss(cont_state_mat, cont_input_mat, [1,0;0,1], []);


Ad_matlab = c2d(sys, T);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Krause's dynamics %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
g = 9.81;
h = 0.814;
T = 0.1;
N = 16;
omega = sqrt(g/h);
% Continuous model
A_cp = [0, 1; omega^2, 0];
b_cp = [0; -omega^2];
c_cp = [1, 1/omega];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% prediction matrixes %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% first we create matrixes which contain the different sampling models
disp('build matrices...')
Acp_all_pos = [];
bcp_all_pos = [];
Acp_all_vel = [];
bcp_all_vel = [];
for i = 1:N
    Ad = expm(A_cp * T * i);
    Bd = inv(A_cp) * (Ad - 1) * b_cp;
    Acp_all_pos = [Acp_all_pos; Ad(1,:)];
    bcp_all_pos = [bcp_all_pos; Bd(1,:)];                                     
    Acp_all_vel = [Acp_all_vel; Ad(2,:)];
    bcp_all_vel = [bcp_all_vel; Bd(2,:)];
end

kd = -1;
q = (1-kd)/(omega^2*kd);