function [Pol,Val,X,A,R] = MDP_FinalProject(P,nAgents,nTargets,N,DynamicModel,Pkp1k)
% Solves the problem using Value Iteration
% State vector x:
% x=[T1...Tnt,A1....Ana]', i.e. the position of nt Targets and then
% position of na Agents
% X is the state space

% Reward:
% Intersection: 100
% Loitering penalty: -1
% More than 1 agent in the same state : -50*(n-1), wheren is the number of
% agents in the same state

% Transition matrix:
% P(s'|s,a) -->  P{a}(s,s')
% a = 1 - stay
% a = 2 - up, a = 3 - down
% a = 4 - right, a = 5 - left


% DynamicModel:  1 - random walk, 2 - still target


% Gives target cell in every time step.
gamma = 0.95;  % Discount factor
epsU = 1e-3;

% Build state-space:
v = 1:N;
str = 'v';
for ii=1:nAgents+nTargets-1
    str = [str,',v'];
end
X=eval(['combvec(',str,')']);


% Build Action-space:
v = 1:5;
str = 'v';
for ii=1:nAgents-1
    str = [str,',v'];
end
A=eval(['combvec(',str,')']);

% Build Reward 
R = 0*ones(size(X,2),1);
interCount = zeros(nTargets,1); % tracking which target had been caught

for jj=1:size(X,2)
    x = X(:,jj); 
    for ii=nTargets+1:nAgents+nTargets
        if sum(x(ii)== x(1:nTargets))   % intersection
            if interCount(find(x(ii)==x(1:nTargets)))==0
                R(jj)=R(jj)+100;
                if sum(interCount~=0)
                    R(jj)=R(jj)+50;
                    targetFlag=1;     % all targets had been caught
                end
            else
                R(jj)=R(jj)+10;     
            end
            interCount(find(x(ii)==x(1:nTargets))) = interCount(find(x(ii)==x(1:nTargets)))+1;
                
        elseif sum(x(ii)== x(ii+1:nAgents+nTargets))  % 2 agents in the same cell
                R(jj)=R(jj)-10;   
        else   % loitering
            R(jj)=R(jj)-1;
        end      
    end
    interCount = zeros(nTargets,1);
end

% S=sparse(T{a});
%  [i,j,k]=find(S);

% Utility
U = min(R(:,1))*ones(size(R));

k=1;
deltaU=1;
while deltaU(k)>epsU
    
    for s=1:size(X,2)  % current state
        Utmp = zeros(1,size(A,2));
        [T] = BuildTransitionMatrix2(s,X,nAgents,nTargets,P,A,DynamicModel,Pkp1k);
        for a=1:size(A,2)   % actions
            for sp1 = 1:size(X,2) % next state
                  Utmp(1,a) = Utmp(1,a)+T(a,sp1).*U(sp1,k);
            end
        end
        [U(s,k+1),Pol(s,1)] = max(R(s,1)+gamma*Utmp(1,:),[],2);
    end
   
    k=k+1; 
    deltaU(k) = max(abs(1-U(:,k)./U(:,k-1)));
    
    if k>1000
        break
    end
    
    
    
    
end

Val = U(:,end);
 


end