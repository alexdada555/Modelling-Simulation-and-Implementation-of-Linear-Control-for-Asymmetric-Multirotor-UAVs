%%% Find a MAS given the following dynamics
%%%     F x <=t
%%% Process is x(k+1)=Ax(k)
%%% Constraints at each sample are Cx <=f
%%%
%%% Uses a simple while loop and simple method based 
%%% on gradually increasing the number of rows of a matrix inequality
%%% does not remove redundant constraints
%%%  F=[C;CA;CA^2;...] <=[f;f;f;...]
%%%
%%%  [F,t]=findmas(A,C,f)

% File produced by Anthony Rossiter (University of Sheffield)
% With a creative commons copyright so that users can re-use and edit and redistribute as they please.
% Files are deliberately simple so users can more easily follow the code and edit as required.
% Provided free of charge and thus with no warranty. 

function [F,t]=findmas(A,C,f)

%%%% initial set for k=0;
F=C;
t=f;
val=zeros(size(f));
An=C;
nc=size(C,1);  %%% number of constraints in each sample
Inc=eye(nc);

%%% Switch of display
opt = optimset('linprog');
opt.Diagnostics='off';    %%%%% Switches of unwanted MATLAB displays
%opt.LargeScale='off';     %%%%% However no warning of infeasibility
opt.Display='off';

cont=1;
while cont==1;
    An=An*A;   %%% forms new block An=C*A^n
    
    %%% inequalities to maximise - check each row of new constraints
    %%%  e(j)^T An x <= t(j)
    for j=1:nc;
        vec=-Inc(j,:)*An;
        [x,vv,exitflag]=linprog(vec,F,t,[],[],[],[],[],opt);
        if exitflag==1; val(j)=vv;
        elseif exitflag==-3; %%% indicates solution unbounded
            val(j)=-f(j)*2; %% marks this row as needed
        end
    end
   
    %%% Extra rows are needed if any values of val exceed f
    %%% so add them in
    if any(-val>f);
        
    F=[F;An];  %% Add extra block to F
    t=[t;f];   %% Add extra block to t
        else
        cont=0;  %%% finish loop as all new rows redundant
    end
end

    