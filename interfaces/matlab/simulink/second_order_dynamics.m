clear;
%% Constants
g = 9.81;
h = 0.814;
T = 0.1;
N = 16;
omega = sqrt(g/h);

%% Dynamics
%%% Continuous model
%%%% Third order model (not tested!)
%Ac = [0, 1, 0; 0, 0, 1; 0, 0, 0];
%Bc = [0; 0; 1];
%C = [1; 0; 0;];          %position
%%%% Second order model
Ac = [0, 1; omega^2, 0];
Bc = [0; -omega^2];
%Ac = [-omega, omega; 0, omega];
%Bc = [0; -omega];
%Ac = [-omega, 0; 0, omega];
%Bc = [omega; -omega];
%C = [1/2, 1/2];          %position
C = [0, 1];              %capture point

%% Prediction matrices for a constant sampling rate
%%% Decoupled modes and standard (ss, c2d, powers of A)

csys = ss(Ac, Bc, C, []);
Ad_vec = [];
dsys = c2d(csys, T);

%%%% Standard form
S = zeros(N, 2);
U = zeros(N, N);
% Fill state matrix and main diagonal of input matrix
for i = 1:N
   S(i,:) = dsys.C * dsys.A;
   U(i,i) = dsys.C * dsys.B;
end

% Fill lower triangle of input matrix
for i = 1:N-1
    CAB = dsys.C * dsys.A^i * dsys.B;
    %fill diagonal i+1
    for j = 1:N-i
        U(i+j, j) = CAB;
    end
end
H = U'*U;
disp('Condition number (standard form): ');
disp(cond(H));

%%%% Decoupled form (Goodwin - Chapter11)
As = dsys.A(1,1);
Au = dsys.A(2,2);
for i = 1:N
   Ad_vec = [Ad_vec; [As^i, 0; 0, Au^(-i)]]; %powers of A
end

S_s = zeros(N, 1);
S_u = zeros(N, 1);
U_s = zeros(N, N);
U_u = zeros(N, N);
U_s(1:N+1:end) = dsys.C(1) * dsys.B(1);
for i = 1:N-1
    CABs = dsys.C(1) * Ad_vec(2*i-1, 1) * dsys.B(1);
    CABu = -dsys.C(2) * Ad_vec(2*i, 2) * dsys.B(2);
    %fill diagonal i+1
    for j = 1:N-i
        U_s(i+j, j) = CABs;
        U_u(j, i+j) = CABu;
    end
end
U = U_s + U_u;
H = U'*U;
disp('Condition number of regularized sub hessian (decoupled modes): ');
disp(cond(H));