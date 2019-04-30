function [Q] = POMDP_FinalProject(P,nAgents,nTargets,X,DynamicModel,vMDP,A,R,pyxCell,yMeas,AlgoFlag,S_A)
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

% yMeas{ii,jj}(nn,kk) is the measurement taken by agent ii of target jj
% when the agent is in cell number nn and the time step is kk

% Gives target cell in every time step.
gamma = 0.95;  % Discount factor
epsU = 1e-5;



% Utility
% U = min(R(:,1))*ones(size(R));


% QMDP  
if AlgoFlag==1 %QMDP
    for s=1:size(X,2)  % current state
        Qtmp = zeros(1,size(A,2));
        for a=1:size(A,2)   % actions
            [T] = BuildTransitionMatrix2(s,a,X,nAgents,nTargets,P,A,DynamicModel);
            for sp1 = 1:size(X,2) % next state
                  Qtmp(1,a) = Qtmp(1,a)+T(1,sp1).*vMDP(sp1);
            end
            Q(s,a) = R(s,1)+Qtmp(1,a);
        end

    end
else % FIB
    for s=1:size(X,2)  % current state
        Qtmp = zeros(1,size(A,2));
        [T] = BuildTransitionMatrix2(s,X,nAgents,nTargets,P,A,DynamicModel);
        for a=1:size(A,2)   % actions
            
            for sp1 = 1:size(X,2) % next state
                for o=1:nObs
                  Qtmp(1,a) = Qtmp(1,a)+pyx*T(a,sp1).*vMDP(sp1);
            end
            Q(s,a) = R(s,1)+Qtmp(1,a);
        end

    end
   
    end 
end



% Bayes update
    % step k=0;
p0 = ones(size(X,2),1)/size(X,2);
[~,a] = max(repmat(p0,1,size(A,2)).*Q,[],2);

% for s=1:size(X,2)  % current state
%         [T] = BuildTransitionMatrix2(s,a(s,1),X,nAgents,nTargets,P,A,DynamicModel);
%         % Prediction
%         pxy_m(s,1)=T*p0;
% end

pmx = eye(size(pyxCell));  % agent likelihood of own state m
for kk=1:legth(t)
    % Building the observation vector
    % Sk is the state at time step k
    jjj=1;
    for ii=1:nAgents
            for jj=1:nTargets
                O(jjj,1)=yMeas{ii,jj}(Sk(ii+nTargets,1),kk);
                jjj=jjj+1;
            end
    end
    for ii=1:nAgents
        O(ii+nTargets*nAgents,1)=mMeas(ii,kk);
    end
    
    % Building the likelihood vector of the given observation
    pOX = ones(size(X,2),1);
    for s=1:size(X,2)
        jjj=1;
        for ii=1:nAgents
            for jj=1:nTargets
                if O(jjj,1)==0
                    P_Like =1-pyxCell(X(jj,s),X(ii+nTargets,s));
                else
                    P_Like =pyxCell(X(jj,s),X(ii+nTargets,s));
                end

                pOX(s,1) = pOX(s,1)*pmx(O(ii+nTargets*nAgents,1),X(ii+nTargets,s))*P_Like;

            end
        end
    end
    
    
    % Bayes update
        
    % step k=1
    for s=1:size(X,2)  % current state
        [T] = BuildTransitionMatrix2(s,a(s,1),X,nAgents,nTargets,P,A,DynamicModel,Pkp1k);
        % Prediction
        if kk==1
            pxy_m(s,1)=T*p0;
        else
            pxy_m(s,1)=T*pxy_p(s,1);
        end
        
        % Bayesian measurment update
        pxy_p(s,1) = pxy_m(s,1)*pOX(s,1);
        
        
    end
%     Normalize
    pxy_p = pxy_p/pxy_m'*pOX;
    
    [~,a] = max(repmat(pxy_p,1,size(A,2)).*Q,[],2);


 


end