function [H,L,M] = cost_mats(F,G,Q,R,P)
%
% COST_MATS.M returns the MPC cost matrices for a system
%
%	X = F*x + G*U
%
% with finite-horizon objective function
%
%	J = 0.5*sum_{j=0}^{N-1} x(j)'*Q*x(j) + u(j)'*R*u(j) + x(N)'*P*x(N)
%
% That is, the matrices H, L and M in
%
%	J = 0.5*U'*H*U + x'*L'*U + x'*M*x
%
% USAGE:
%
% 	[H,L,M] = cost_mats(F,G,Q,R,P)
%
% P. Trodden, 2015.

% get dimensions
n = size(F,2);
N = size(F,1)/n;

% diagonalize Q and R
Qd = kron(eye(N-1),Q);
Qd = blkdiag(Qd,P);
Rd = kron(eye(N),R);

% Hessian
H = 2*G'*Qd*G + 2*Rd;

% Linear term
L = 2*G'*Qd*F;

% Constant term
M = F'*Qd*F + Q;

% make sure the Hessian is symmetric
H=(H+H')/2;
