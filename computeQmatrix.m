function [Q] = computeQmatrix(P,nAgents,nTargets,X,DynamicModel,vMDP,A,R,AlgoFlag)


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
            end
            Q(s,a) = R(s,1)+Qtmp(1,a);
            
        end

    end
   
end


end
