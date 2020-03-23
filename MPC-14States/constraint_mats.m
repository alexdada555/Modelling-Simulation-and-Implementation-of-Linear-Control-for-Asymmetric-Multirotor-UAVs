function [Pc, qc, Sc] = constraint_mats(F,G,Pu,qu,Px,qx,Pxf,qxf)
%
% CONSTRAINTS_MATS.M returns the MPC constraints matrices for a system that
% is subject to constraints
%
%    Pu*u(k+j|k) <= qu
%    Px*x(k+j|k) <= qx
%    Pxf*x(k+N|k) <= qxf
%
% That is, the matrices Pc, qc and Sc from
%
%   Pc*U(k) <= qc + Sc*x(k)
%
% USAGE:
%
%   [Pc,qc,Sc] = constraint_mats(F,G,Pu,qu,Px,qx,Pxf,qxf)
%
% where F, G are the prediction matrices obtained from PREDICT_MATS.M
%
% P. Trodden, 2017.

% input dimension
m = size(Pu,2);

% state dimension
n = size(F,2);

% horizon length
N = size(F,1)/n;

% number of input constraints
ncu = numel(qu);

% number of state constraints
ncx = numel(qx);

% number of terminal constraints
ncf = numel(qxf);

% if state constraints exist, but terminal ones do not, then extend the
% former to the latter
if ncf == 0 && ncx > 0
    Pxf = Px;
    qxf = qx;
    ncf = ncx;
end

%% Input constraints

% Build "tilde" (stacked) matrices for constraints over horizon
Pu_tilde = kron(eye(N),Pu);
qu_tilde = kron(ones(N,1),qu);
Scu = zeros(ncu*N,n);

%% State constraints

% Build "tilde" (stacked) matrices for constraints over horizon
Px0_tilde = [Px; zeros(ncx*(N-1) + ncf,n)];
if ncx > 0
    Px_tilde = [kron(eye(N-1),Px) zeros(ncx*(N-1),n)];
else
    Px_tilde = zeros(ncx,n*N);
end
Pxf_tilde = [zeros(ncf,n*(N-1)) Pxf];
Px_tilde = [zeros(ncx,n*N); Px_tilde; Pxf_tilde];
qx_tilde = [kron(ones(N,1),qx); qxf];

%% Final stack
if isempty(Px_tilde)
    Pc = Pu_tilde;
    qc = qu_tilde;
    Sc = Scu;
else
    % eliminate x for final form
    Pc = [Pu_tilde; Px_tilde*G];
    qc = [qu_tilde; qx_tilde];
    Sc = [Scu; -Px0_tilde - Px_tilde*F];
end
