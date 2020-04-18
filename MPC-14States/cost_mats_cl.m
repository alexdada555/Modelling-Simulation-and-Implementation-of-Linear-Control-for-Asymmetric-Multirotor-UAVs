function [Sc,Scx] = cost_mats_cl(Fxcl,Gxcl,Fycl,Gycl,Fucl,Gucl,Q,R,P)
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
%   J = = cT Sc c + 2cT Scx x + k
%   Sc = HTc diag(Q)Hc + HTcu diag(R) Hcu + HTc2 P Hc2
%   Scx = HTc diag(Q)Pcl + HTcu diag(R)Pclu + HTc2 P Pcl2
%
% USAGE:
%
% 	[H,L,M] = cost_mats(F,G,Q,R,P)
%   [H,L,M,Sc,Scx] = cost_mats_cl(Fxcl,Gxcl,Fycl,Gycl,Fucl,Gucl,Q,R,P)
%
% P. Trodden, 2015.

% get dimensions
n = size(Fxcl,2);
N = size(Fxcl,1)/n;

% diagonalize Q and R
Qd = kron(eye(N-1),Q);
Qd = blkdiag(Qd,P);
Rd = kron(eye(N),R);

% Hessian
Sc = 2*Gxcl'*Qd*Gxcl + 2*Gucl'*Rd*Gucl;

% make sure the Hessian is symmetric
Sc = (Sc+Sc')/2;

% Linear term
Scx = 2*Gxcl'*Qd*Fxcl + 2*Gucl'*Rd*Fucl;

