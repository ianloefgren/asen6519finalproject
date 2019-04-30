function [T] = BuildTransitionMatrix2(s,a,X,nAgents,nTargets,P,A,DynamicModel)
%% State Transition Matrix
% P(s'|s,a) -->  P{a}(s,s')
% a = 1 - stay
% a = 2 - up, a = 3 - down
% a = 4 - right, a = 5 - left





T = zeros(1,size(X,2));

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
    
    if DynamicModel==2  % still target  - % Only states where target states are stationary
%         if nAgents+nTargets>2
            [row,stateInd] = find(sum(X==[X(1:nTargets,s) ; TmpStateVec(:,jj)])==(nTargets+nAgents));
%         else
%             [row,stateInd] = find(X==[X(1:nTargets,s) ; TmpStateVec(:,jj)]);
%         end
    else
     % All possible state wehre the targets are in different cells but the
    % agents states are constant
        if nAgents>1
        [row,stateInd] = find(sum(X(nTargets+1:end,:)==TmpStateVec(:,jj))==nAgents);
            else
        [row,stateInd] = find(X(nTargets+1:end,:)==TmpStateVec(:,jj));
        end
    end
    % Only states where target states are stationary
%     if DynamicModel==2  % still target
%         stateInd = stateInd(find(X(1:nTargets,stateInd)==X(1:nTargets,s)));                  
% 
%     end 
    if sum(T(1,stateInd))==0
        T(1,stateInd)=1;
    end
    for nn=1:nAgents                
        T(1,stateInd) = T(1,stateInd)*Ptmp(nn,TmpStateVec(nn,jj));
    end
end
if sum(T(1,:))==0
    1;
end
T(1,:) = T(1,:)/sum(T(1,:));  % normalize


