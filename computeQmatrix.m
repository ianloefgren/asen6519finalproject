function [Q] = computeQmatrix(P,nAgents,nTargets,X,DynamicModel,vMDP,A,R,AlgoFlag,Pkp1k,S_A,pOjX,pmx)


% QMDP  
if AlgoFlag==1 %QMDP
    for s=1:size(X,2)  % current state
        Qtmp = zeros(1,size(A,2));
        [T] = BuildTransitionMatrix2(s,X,nAgents,nTargets,P,A,DynamicModel,Pkp1k);
        for a=1:size(A,2)   % actions
            for sp1 = 1:size(X,2) % next state
                  Qtmp(1,a) = Qtmp(1,a)+T(a,sp1).*vMDP(sp1);
            end
            Q(s,a) = R(s,1)+Qtmp(1,a);
        end

    end
else % FIB
%     for s=1:size(X,2)  % current state
%         Qtmp = zeros(1,size(A,2));
%         [T] = BuildTransitionMatrix2(s,X,nAgents,nTargets,P,A,DynamicModel,Pkp1k);
%         for a=1:size(A,2)   % actions
%             
%             for sp1 = 1:size(X,2) % next state
%                 for o=1:nObs
%                   Qtmp(1,a) = Qtmp(1,a)+pyx*T(a,sp1).*vMDP(sp1);
%                 end
%             end
%             Q(s,a) = R(s,1)+Qtmp(1,a);
%             
%         end


    % create enumeration of possible observations
    Y = combvec(0:1,0:1,1:size(S_A,2),1:size(S_A,2));

    for s=1:size(X,2)
        % create tmp vector for Q values of each action
        Qtmp = zeros(1,size(A,2));
        % build transition matrix for all actions
        T = BuildTransitionMatrix2(s,X,nAgents,nTargets,P,A,DynamicModel,Pkp1k);
        % iterate over possible actions
        for a=1:size(A,2)
            
            % find possible next states where T != 0
            possible_states = find(T(a,:));
            % iterate over possible next states
            for sp = possible_states
                
                % iterate over possible observations
                for o=1:size(Y,2)
                    
                    % Building the likelihood vector of the given observation
                    obs_likelihood = ones(size(X,2),1);

                    for sA=1:size(S_A,2)
                        if nAgents>1
                            % find the agents state # in the agent-state space that co
                            [row,s_ind] = find(sum(S_A(:,sA)==X(nTargets+1:end,:))==nAgents);
                        else
                            [row,s_ind] = find(S_A(:,sA)==X(nTargets+1:end,:));
                        end
                        for ii=1:nAgents
                            for jj=1:nTargets
                                if Y(jj,o)==0
                                    P_Like =1-pOjX(:,sA);
                                else
                                    P_Like =pOjX(:,sA);
                                end

                                obs_likelihood(s_ind,1) = obs_likelihood(s_ind,1).*pmx(Y(ii+nTargets,o),S_A(ii,sA)).*P_Like;

                            end
                        end
                    end
                    
                    Qtmp(1,a) = Qtmp(1,a) + obs_likelihood(sp).*T(a,sp).*vMDP(sp);
                end
            end
            
            
        end
        
        % add potential future utility  and current reward to Q fxn
        Q(s,a) = R(s,1) + Qtmp(1,a);
            
            
    end
    
    
end


end
