function [Pol,Val] = MDP_FinalProject(P,Tloc,nAgents,nTargets,N)
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
% a = 1 - up, a = 2 - down
% a = 3 - right, a = 4 - left
% a = 5 - stay

% Tloc - Target location:
% Gives target cell in every time step.
gamma = 0.95;  % Discount factor

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


for s=1:size(X,2)  % States
    for a=1:size(A,2)  % Actions
        if s==1
            T{a} = zeros(size(X,2),size(X,2));
        end
        stateSize=1;
        str = '';
        for ii= 1:nAgents
            Ptmp(ii,:) = P{A(ii,a)}(X(ii+nTargets,s),:);
            indCell{ii} = find(Ptmp(ii,:)~=0);
            stateSize = stateSize*size(indCell{ii},2);
            str = [str,['indCell{',num2str(ii),'},']];
        end
        
        TmpStateVec = eval(['combvec(',str(1:end-1),')']);
        
        
            for jj=1:size(TmpStateVec,2)
                if nAgents>1
                    [row,stateInd] = find(sum(X(nTargets+1:end,:)==TmpStateVec(:,jj))==nAgents);
                else
                    [row,stateInd] = find(X(nTargets+1:end,:)==TmpStateVec(:,jj));
                end
                if sum(T{a}(s,stateInd))==0
                    T{a}(s,stateInd)=1;
                end
                for nn=1:nAgents                
                    T{a}(s,stateInd) = T{a}(s,stateInd)*Ptmp(1,TmpStateVec(nn,jj));
                end
            end
        
    end
end


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
                R(jj)=R(jj)-50;   
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

for k=1:1000
    Utmp = zeros(1,size(A,2));
    for s=1:size(X,2)  % current state
        for a=1:size(A,2)   % actions
            for sp1 = 1:size(X,2) % next state
                Utmp(1,a) = Utmp(1,a)+T{a}(s,sp1)*U(sp1,k);
            end
        end
        U(s,k+1) = R(s,1)+gamma*max(Utmp(1,:),[],2);
    end
   
    
end





end