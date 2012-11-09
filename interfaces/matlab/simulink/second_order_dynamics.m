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

%% Constants
g = 9.81;
h = 0.814;
T = 0.1;
N = 16;
omega = sqrt(g/h);

%% Dynamics
% Continuous model
%A_cp = [0, 1; omega^2, 0];
%b_cp = [0; -omega^2];
%A_cp = [-omega, omega; 0, omega];
%b_cp = [0; -omega];
%A_cp = [0, 1, 0; 0, 0, 1; 0, 0, 0];
%b_cp = [0; 0; 1];
A_cp = [-omega, 0; 0, omega];
B_cp = [omega; -omega];
C_pos = [1/2, 1/2];

%% Prediction matrices
S_pos = [];
U_pos = [];
for i = 1:N
    Ad = expm(A_cp * T * i);
    Bd = inv(A_cp) * (Ad - eye(2,2)) * B_cp;
    S_pos = [S_pos; C_pos * Ad];
    U_pos = [U_pos; C_pos * Bd];
end
S_pos;
U_pos;

S_pos = [];
U_pos = [];
Ad = expm(A_cp * T);
Bd = inv(A_cp) * (Ad - eye(2,2)) * B_cp;
for i = 1:N
    S_pos = [S_pos; C_pos * Ad^i];
    U_pos = [U_pos; C_pos * Ad^(i-1)*Bd];
end
S_pos;
U_pos

S_pos_st = [];
S_pos_unst = [];
U_pos_dec = zeros(N,N);
for i = 1:N
    Ad(1,1) = exp(A_cp(1,1) * T * i);
    Ad(2,2) = exp(-A_cp(2,2) * T * i);
    Bd = inv(A_cp) * (Ad - eye(2,2)) * B_cp;
    S_pos_st = [S_pos_st; C_pos(1,1) * Ad(1,1)];
    S_pos_unst = [S_pos_unst; C_pos(1,2) * Ad(2,2)];
    U_pos_dec(i,1) = C_pos(1,1) * Bd(1);
    U_pos_dec(1,i) = C_pos(1,2) * Bd(2);
end
U_pos_dec;

S_pos_st = [];
S_pos_unst = [];
U_pos_dec = zeros(N, N);
Ad(1,1) = exp(A_cp(1, 1)*T);
Ad(2,2) = exp(-A_cp(2, 2)*T);
Bd = inv(A_cp) * (Ad - eye(2,2)) * B_cp;
for i = 1:N
    Adi = Ad^i;
    Adii = Ad^(i-1);
    Ui = Adii * Bd;
    S_pos_st = [S_pos_st; C_pos(1,1) * Adi(1,1)];
    S_pos_unst = [S_pos_unst; C_pos(1,2) * Adi(2,2)];
    U_pos_dec(i,1) = C_pos(1,1) * Ui(1);
    U_pos_dec(1,i) = C_pos(1,2) * Ui(2);
end
U_pos_dec;

%% PID
kd = -1;
q = (1-kd)/(omega^2*kd);