function [Pol,Val,X,A,R] = MDP_FinalProject(P,nAgents,nTargets,N,DynamicModel,Pkp1k,L)
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

%% Penalizing for trying to get out of the grid
R=repmat(R,1,size(A,2));

for ii=1:nAgents
    % left bound
    if nAgents>1
        [row,stateInd] = find(sum(X(nTargets+ii,:)<=L)==nAgents);
    else
        [row,stateInd] = find(X(nTargets+ii,:)<=L);
    end
    R(stateInd,5)=R(stateInd,5)-200;
    
    % Right Bound
    if nAgents>1
        [row,stateInd] = find(sum(X(nTargets+ii,:)>(N-L))==nAgents);
    else
        [row,stateInd] = find(X(nTargets+ii,:)>(N-L));
    end
    R(stateInd,4)=R(stateInd,4)-200;
    
    % upper bound
    for ll=1:L
        if nAgents>1
            [row,stateInd] = find(sum(X(nTargets+ii,:)==L*ll)==nAgents);
        else
            [row,stateInd] = find(X(nTargets+ii,:)==L*ll);
        end
        R(stateInd,2)=R(stateInd,2)-200;
    end
    
    % lower bound
    for ll=1:L
        if nAgents>1
            [row,stateInd] = find(sum(X(nTargets+ii,:)==1+L*(ll-1))==nAgents);
        else
            [row,stateInd] = find(X(nTargets+ii,:)==1+L*(ll-1));
        end
        R(stateInd,3)=R(stateInd,3)-200;
    end
end

% Utility
U = min(R(:,1))*ones(size(R));

total_iter_time = 0;

k=1;
deltaU=1;
while deltaU(k)>epsU
    
    tic;
    
    for s=1:size(X,2)  % current state
        Utmp = zeros(1,size(A,2));
        [T] = BuildTransitionMatrix2(s,X,nAgents,nTargets,P,A,DynamicModel,Pkp1k);
        for a=1:size(A,2)   % actions
            for sp1 = 1:size(X,2) % next state
                  Utmp(1,a) = Utmp(1,a)+R(s,a)+gamma*T(a,sp1).*U(sp1,k);
            end
        end
         [U(s,k+1),Pol(s,1)] = max(Utmp(1,:),[],2);

    end
   
    k=k+1; 
    deltaU(k) = max(abs(1-U(:,k)./U(:,k-1)));
    
    iter_time = toc;
    total_iter_time = total_iter_time + iter_time;
    avg_iter_time = total_iter_time / k;
    
    clc;
    fprintf('=============================================================\n');
    fprintf('MDP Value Iteration: Solving value function...\n')
    fprintf('-------------------------------------------------------------\n')
    fprintf('Num iterations: %i \t Total run time: %0.2f (s)\n',k,total_iter_time)
    fprintf('Last iteration time: %0.2f (s)\t Avg. iteration time: %0.2f (s)\n',iter_time,avg_iter_time)
    fprintf('Last max deltaU: %0.4f \t covergence epsU: %0.4f\n',deltaU(k),epsU);
    fprintf('=============================================================\n');
    
    if k>1000
        break
    end
    
    
    
    
end

Val = U(:,end);
 


end