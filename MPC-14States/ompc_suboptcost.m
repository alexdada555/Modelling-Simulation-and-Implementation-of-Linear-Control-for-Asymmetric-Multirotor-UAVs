%%%  Determine suboptimal MPC cost, including bits not depending on d.o.f.
%%%  nc is the control horizon.
%%%  Assumes terminal mode not matched to the LQR regulator.
%%%  for performance index of    J =sum x'Q2 x+u'R2 u
%%%
%%%  Assume model x(k+1)=Ax(k)+Bu(k), y= Cx+Du
%%%
%%%  Overall cost
%%%% J = x SX x + 2*x SXC c + c SC c
%%
%%%%% Terminal Control law u=-Kx and predictions based on Q2, R2
%%%%% Regulation case only based on LQR feedback for
%%%%%     J =sum x'Q2 x+u'R2 u
%%%%
%%%%   [SX,SC,SXC,Spsi,K,Psi,Kz]=chap4_suboptcost(A,B,C,D,Q,R,Q2,R2,nc)
%%%%
%%%%  Builds an autonomous model Z= Psi Z, u = -Kz Z  Z=[x;cfut];
%%%%
%%%% Code by JA Rossiter (2014)

% File produced by Anthony Rossiter (University of Sheffield)
% With a creative commons copyright so that users can re-use and edit and redistribute as they please.
% Files are deliberately simple so users can more easily follow the code and edit as required.
% Provided free of charge and thus with no warranty. 


function [SX,SC,SXC,Spsi,K,Psi,Kz]=chap4_suboptcost(A,B,C,D,Q,R,Q2,R2,nc)

nx = size(A,1);
nxc=nx*nc;
nu=size(B,2);
nuc=nu*nc;

%%%%%  Feedback loop is of the form  u = -Kx+c
%%%%%  Find terminal mode feedback (uses LQR based on Q2, R2) 
[K] = dlqr(A,B,Q2,R2);
Phi=A-B*K;

%%% Build autonomous model
ID=diag(ones(1,(nc-1)*nu));
ID=[zeros((nc-1)*nu,nu),ID];
ID=[ID;zeros(nu,nuc)];
Psi=[A-B*K,[B,zeros(nx,nuc-nu)];zeros(nuc,nx),ID];
Gamma=[eye(nx),zeros(nx,nuc)];
Kz = [K,-eye(nu),zeros(nu,nuc-nu)];

%%%% Solve for the cost parameters using lyapunov
W=Psi'*Gamma'*Q*Gamma*Psi+Kz'*R*Kz;
Spsi=dlyap(Psi',W);

%%% Split cost into parts
%%%% J = [x,c] Spsi [x;c]
%%%% J = x SX x + 2*x SXC c + c SC c
SX=Spsi(1:nx,1:nx);
SXC=Spsi(1:nx,nx+1:end);
SC=Spsi(nx+1:end,nx+1:end);




