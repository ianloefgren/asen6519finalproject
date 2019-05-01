function [Q] = POMDP_FinalProject(P,nAgents,nTargets,X,DynamicModel,vMDP,A,R,pOjX,pmx,...
    AlgoFlag,S_A,Pkp1k,xTrue,seed)
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

% xTrue is the true state

% Gives target cell in every time step.
gamma = 0.95;  % Discount factor
epsU = 1e-5;

i=1;

num_moves(i) = 0;
% Utility
% U = min(R(:,1))*ones(size(R));

[Q] = computeQmatrix(P,nAgents,nTargets,X,DynamicModel,vMDP,A,R,AlgoFlag,Pkp1k);

caught_flags = zeros(1,nTargets);
p0 = zeros(size(X,2),1);  

% Bayes update
    % step k=0;

if nAgents>1
    % find the agents state # in the state space that correspond to the true state
    [row,stateInd] = find(sum(X(nTargets+1:end,:)==xTrue(nTargets+1:end,1))==nAgents);
    % find the agents state # in the agent-state space that correspond to the true state
    [row,stateIndAgent] = find(sum(S_A==xTrue(nTargets+1:end,1))==nAgents);
else
    [row,stateInd] = find(X(nTargets+1:end,:)==xTrue(nTargets+1:end,1));
    [row,stateIndAgent] = find(S_A==xTrue(nTargets+1:end,1));
end
p0(stateInd,1) = 1/size(stateInd,2);
[~,a] = max(sum(repmat(p0,1,size(A,2)).*Q),[],2);
num_moves(i) = num_moves(i) + 1;

% find the current state # in thestate space
[row,s] = find(sum(X==xTrue)==(nAgents+nTargets));
% Transition from s to s' given the action a that we chose 
T= BuildTransitionMatrix2(s,X,nAgents,nTargets,P,A(a),DynamicModel,Pkp1k);        

sTrue(1) = randsample(size(X,2),1,true,T);
 



% Sample mesurements
for jj=1:nTargets 
    Y(jj,1) = randsample(seed,2,1,true,[1-pOjX(X(jj,sTrue),stateIndAgent) pOjX(X(jj,sTrue),stateIndAgent)])-1;
end
for jj=nTargets+1:nTargets+nAgents
    Y(jj,1) = randsample(seed,size(pmx,1),1,true,pmx(:,stateIndAgent) ,stateIndAgent);
end


%  Propagate belief
pxy_m = zeros(size(p0));

% Prediction
for s=1:size(X,2)
    T= BuildTransitionMatrix2(s,X,nAgents,nTargets,P,A(a),DynamicModel,Pkp1k);   
    pxy_m(:,1)=pxy_m(:,1)+T'*p0(s,1);
end

% Building the likelihood vector of the given observation
pYX = ones(size(X,2),1);


for sA=1:size(S_A,2)
    if nAgents>1
        % find the agents state # in the agent-state space that co
        [row,s] = find(sum(S_A(:,sA)==X(nTargets+1:end,:))==nAgents);
    else
        [row,s] = find(S_A(:,sA)==X(nTargets+1:end,:));
    end
    for ii=1:nAgents
        for jj=1:nTargets
            if Y(jj,1)==0
                P_Like =1-pOjX(:,sA);
            else
                P_Like =pOjX(:,sA);
            end

            pYX(s,1) = pYX(s,1).*pmx(Y(ii+nTargets,1),S_A(ii,sA)).*P_Like;

        end
    end
end
% Bayesian measurment update
pxy_p = pxy_m.*pYX/(pxy_m'*pYX);

[~,a] = max(sum(repmat(pxy_p,1,size(A,2)).*Q),[],2);

num_moves(i) = num_moves(i) + 1;
trajectories{i}(:,1) = [sTrue(1);a;caught_flags'];
belief{i}(:,1) = pxy_p;

% record action state pair, reward, and move count
% trajectories{i}(:,end+1) = [s;action;caught_flags'];
% cum_reward(i) = cum_reward(i) + utility(s);
% num_moves(i) = num_moves(i) + 1;
kk=2;
while ~all(caught_flags)
% Transition from s to s' given the action a that we chose 
    T= BuildTransitionMatrix2(sTrue(kk-1),X,nAgents,nTargets,P,A(a),DynamicModel,Pkp1k);

    sTrue(kk) = randsample(size(X,2),1,true,T);

    for j=1:nAgents
       for k=1:nTargets
           if X(k,sTrue(kk)) == X(nAgents + j,sTrue(kk))
               caught_flags(k) = 1;
           end
       end
    end

    if nAgents>1
        % find the agents state # in the agent-state space that correspond to the true state
        [row,stateIndAgent] = find(sum(S_A==X(nTargets+1:end,sTrue(kk)))==nAgents);
    else
%         [row,stateInd] = find(X(nTargets+1:end,:)==xTrue(nTargets+1:end,1));
        [row,stateIndAgent] = find(S_A==X(nTargets+1:end,sTrue(kk)));
    end
    % Sample mesurements
    for jj=1:nTargets 
        Y(jj,kk) = randsample(seed,2,1,true,[1-pOjX(X(jj,sTrue(kk)),stateIndAgent) pOjX(X(jj,sTrue(kk)),stateIndAgent)])-1;
    end
    for jj=nTargets+1:nTargets+nAgents
        Y(jj,kk) = randsample(seed,size(pmx,1),1,true,pmx(:,stateIndAgent) ,stateIndAgent);
    end

    
    %  Propagate belief
    pxy_m(:,kk) = zeros(size(p0));

    % Prediction
    for s=1:size(X,2)
        T= BuildTransitionMatrix2(s,X,nAgents,nTargets,P,A(a),DynamicModel,Pkp1k);   
        pxy_m(:,kk)=pxy_m(:,kk)+T'*pxy_p(s,1);
    end

    % Building the likelihood vector of the given observation
    pYX = ones(size(X,2),1);

    for sA=1:size(S_A,2)
        if nAgents>1
            % find the agents state # in the agent-state space that co
            [row,s] = find(sum(S_A(:,sA)==X(nTargets+1:end,:))==nAgents);
        else
            [row,s] = find(S_A(:,sA)==X(nTargets+1:end,:));
        end
        for ii=1:nAgents
            for jj=1:nTargets
                if Y(jj,1)==0
                    P_Like =1-pOjX(:,sA);
                else
                    P_Like =pOjX(:,sA);
                end

                pYX(s,1) = pYX(s,1).*pmx(Y(ii+nTargets,1),S_A(ii,sA)).*P_Like;

            end
        end
    end
    % Bayesian measurment update
    pxy_p = pxy_m(:,kk).*pYX/(pxy_m(:,kk)'*pYX);

    [~,a] = max(sum(repmat(pxy_p,1,size(A,2)).*Q),[],2);
    num_moves(i) = num_moves(i) + 1;

    
    %
    if kk>1000
       break 
    end
    belief{i}(:,kk) = pxy_p;
    trajectories{i}(:,kk) = [sTrue(kk);a;caught_flags'];
    kk=kk+1;

end

 



end