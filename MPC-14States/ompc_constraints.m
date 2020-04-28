%%% Simulation of dual mode optimal predictive control - REGULATION CASE
%%%  NO INTEGRAL ACTION/OFFSET computation
%%%
%%  [J,x,y,u,c,KSOMPC] = ompc_simulate_constraints(A,B,C,D,nc,Q,R,Q2,R2,x0,runtime,umin,umax,Kxmax,xmax)
%%
%%   Q, R denote the weights in the actual cost function
%%   Q2, R2 are the weights used to find the terminal mode LQR feedback
%%   nc is the control horizon
%%   A, B,C,D are the state space model parameters
%%   x0 is the initial condition for the simulation
%%   J is the predicted cost at each sample
%%   c is the optimised perturbation at each sample
%%   x,y,u are states, outputs and inputs
%%   KSOMPC unconstrained feedback law
%%
%%  Adds in constraint handling with constraints
%%  umin<u < umax   and    Kxmax*x < xmax

% File produced by Anthony Rossiter (University of Sheffield)
% With a creative commons copyright so that users can re-use and edit and redistribute as they please.
% Files are deliberately simple so users can more easily follow the code and edit as required.
% Provided free of charge and thus with no warranty. 

function [J,u,c] = ompc_constraints(A,B,C,D,nc,Q,R,Q2,R2,x,umin,umax,Kxmax,xmax)

%%%%%%%%%% Initial Conditions 
nu = size(B,2);
nx = size(A,1);

%%%%% The optimal predicted cost at any point 
%%%%%     J = c'*SC*c + 2*c'*SCX*x + x'*Sx*x
%%%%  Builds an autonomous model Z= Psi Z, u = -Kz Z  Z=[x;cfut];
%%%%
%%%% Control law parameters
[SX,SC,SXC,Spsi,K,Psi,Kz] = ompc_suboptcost(A,B,C,D,Q,R,Q2,R2,nc);
if norm(SXC)<1e-10; SXC = SXC*0;end
KK = inv(SC)*SXC';
Ksompc = [K+KK(1:nu,:)];

SC = (SC+SC')/2;
%%%%% Define constraint matrices using invariant set methods on
%%%%%  Z = Psi Z  u = -Kz Z  umin<u<umax   Kxmax *x <xmax
%%%%%
%%%%% First define constraints at each sample as G*x<f
%%%%%
%%%%%  Find MAS as M x + N cfut <= f
G = [-Kz;Kz;[Kxmax,zeros(size(Kxmax,1),nc*nu)]];
f = [umax;-umin;xmax]; 

[F,t] = findmas(Psi,G,f);
N = F(:,nx+1:end);
M = F(:,1:nx);

%%%%% Settings for quadratic program
opt = optimset('quadprog');
opt.Diagnostics ='off';    %%%%% Switches of unwanted MATLAB displays
opt.LargeScale ='off';     %%%%% However no warning of infeasibility
opt.Display = 'off';
%opt.Algorithm = 'trust-region-reflective';


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%   SIMULATION

%%%%% Unconstrained control law
%cfast(:,i) = KK*x(:,i);  
%ufast(:,i) = -Ksompc*x(:,i);

%%%% constrained control law
%%%%  N c + Mx <=t
%%%%  J = c'*SC*c + 2*c'*SCX*x 

[c,vv] = quadprog(SC,SXC'*x,N,t-M*x,[],[],[],[],[],opt);
size(c)
u = -K*x + c(1:nu);

%%% update cost

J = x'*SX*x + 2*c'*SXC'*x + c'*SC*c;

end