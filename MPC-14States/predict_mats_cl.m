function [Fxcl,Gxcl,Fycl,Gycl,Fucl,Gucl] = predict_mats_cl(Phi,B,C,K,N)
%
% PREDICT_MATS.M returns the MPC prediction matrices for a system
%
%   phi = A-BK
%   u = -K*x + c
%	x^+ = phi*x + B*c
%   y^+ = C*(phi*x + B*c)
%   u^+ = K*(phi*x + B*c)
%
% That is, the various matrices F and G from the equations
%
%	Xcl = Fxcl*x + Gxcl*c
%   Ycl = Fycl*x + Gycl*c
%   Ucl = Fucl*x + Gucl*c
%
% USAGE:
%
% 	[Fxcl,Gxcl,Fycl,Gycl,Fucl,Gucl] = predict_mats_cl(phi,B,C,K,N)
%
% where N is prediction horizon length.
%
% P. Trodden, 2015.

% dimensions
n = size(Phi,1);
m = size(B,2);
p = size(C,1);

% allocate
Fxcl = zeros(n*N,n);
Fycl = zeros(p*N,n);
Fucl = zeros(m*N,n);
Gxcl = zeros(n*N,m*(N-1));
Gycl = zeros(p*N,m*(N-1));
Gucl = zeros(m*N,m*(N-1));


% form row by row
for i = 1:N
   %  F
   Fxcl(n*(i-1)+(1:n),:) = Phi^i;
   Fycl(p*(i-1)+(1:p),:) = C * (Phi^i);
   Fucl(m*(i-1)+(1:m),:) = -K * (Phi^(i-1));
   
   % G
   % form element by element
   for j = 1:i
       Gxcl(n*(i-1)+(1:n),m*(j-1)+(1:m)) = Phi^(i-j)*B;
       Gycl(p*(i-1)+(1:p),m*(j-1)+(1:m)) = C * (Phi^(i-j))*B;
       Gucl(m*(i-1)+(1:m),m*(j-1)+(1:m)) = K * (Phi^(i-j-1))*B;
   end
end
