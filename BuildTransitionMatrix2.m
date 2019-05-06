function [T] = BuildTransitionMatrix2(s,X,nAgents,nTargets,P,A,DynamicModel,Pkp1k)
%% State Transition Matrix
% P(s_Ai'|s_Ai,a) -->  P{a}(s_Ai,s_Ai')
% a = 1 - stay
% a = 2 - up, a = 3 - down
% a = 4 - right, a = 5 - left
% This matrix is for one agent given one action

% Pkp1k is the dynamic matrix for a target from one cell to the next.

% T is the joint transition matrix from state s to s' given action vector A(:,a);

% The targets and agents dynamics are not coupled, which allows us to
% separte into two transition matrices pT,pA (target and agent,
% rsepectively), s.t T=pT.*pA

% T = zeros(1,size(X,2));
pT = zeros(1,size(X,2));

%% Targets transition matrix - pT


stateSize=1;
str = '';
for ii= 1:nTargets
    Ptmp(ii,:) = Pkp1k(:,X(ii,s))';
    indCell{ii} = find(Ptmp(ii,:)~=0);
%             if DynamicModel==2  % still target
%                 1;
%             end
    stateSize = stateSize*size(indCell{ii},2);
    str = [str,['indCell{',num2str(ii),'},']];
end

TmpStateVec = eval(['combvec(',str(1:end-1),')']);


for jj=1:size(TmpStateVec,2)             


         % All possible state wehre the targets are in different cells but the
        % agents states are constant
        if nTargets>1
        [row,stateInd] = find(sum(X(1:nTargets,:)==TmpStateVec(:,jj))==nTargets);
            else
        [row,stateInd] = find(X(1:nTargets,:)==TmpStateVec(:,jj));
        end

        % Only states where target states are stationary
    %     if DynamicModel==2  % still target
    %         stateInd = stateInd(find(X(1:nTargets,stateInd)==X(1:nTargets,s)));                  
    % 
    %     end
    if DynamicModel==1  % random walk
        if sum(pT(1,stateInd))==0
            pT(1,stateInd)=1;
        end
        for nn=1:nTargets                
            pT(1,stateInd) = pT(1,stateInd)*Ptmp(nn,TmpStateVec(nn,jj));
        end

    elseif DynamicModel==2  % still target

         pT(1,stateInd)=1;
     end
    if sum(pT(1,:))==0
        1;
    end
end
pT(1,:) = pT(1,:)/sum(pT(1,:));  % normalizev

%% Agents transition matrix - pA
for a=1:size(A,2)
    pA = zeros(1,size(X,2));
    stateSize=1;
    str = '';
    for ii= 1:nAgents
        Ptmp(ii,:) = P{A(ii,a)}(X(ii+nTargets,s),:);
        indCell{ii} = find(Ptmp(ii,:)~=0);
    %             if DynamicModel==2  % still target
    %                 1;
    %             end
        stateSize = stateSize*size(indCell{ii},2);
        str = [str,['indCell{',num2str(ii),'},']];
    end

    TmpStateVec = eval(['combvec(',str(1:end-1),')']);


    for jj=1:size(TmpStateVec,2)             

%         if DynamicModel==2  % still target  - % Only states where target states are stationary
%     %         if nAgents+nTargets>2
%                 [row,stateInd] = find(sum(X==[X(1:nTargets,s) ; TmpStateVec(:,jj)])==(nTargets+nAgents));
%     %         else
%     %             [row,stateInd] = find(X==[X(1:nTargets,s) ; TmpStateVec(:,jj)]);
%     %         end
%         else
         % All possible state wehre the targets are in different cells but the
        % agents states are constant
            if nAgents>1
            [row,stateInd] = find(sum(X(nTargets+1:end,:)==TmpStateVec(:,jj))==nAgents);
                else
            [row,stateInd] = find(X(nTargets+1:end,:)==TmpStateVec(:,jj));
            end
%         end
        % Only states where target states are stationary
    %     if DynamicModel==2  % still target
    %         stateInd = stateInd(find(X(1:nTargets,stateInd)==X(1:nTargets,s)));                  
    % 
    %     end 
        if sum(pA(1,stateInd))==0
            pA(1,stateInd)=1;
        end
        for nn=1:nAgents                
            pA(1,stateInd) = pA(1,stateInd)*Ptmp(nn,TmpStateVec(nn,jj));
        end
    end
    if sum(pA(1,:))==0
        1;
    end
    pA(1,:) = pA(1,:)/sum(pA(1,:));  % normalize
    
    T(a,:) = pT.*pA/(pT*pA');
end

