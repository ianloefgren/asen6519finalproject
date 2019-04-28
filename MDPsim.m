% MDP simulator function
%
% Runs simulations fkfkfk
%

function MDPsim(policy,transition_fxn,X,num_sims,num_agents,num_targets,num_states,dyn)

    % Build Action-space:
    v = 1:5;
    str = 'v';
    for ii=1:num_agents-1
        str = [str,',v'];
    end
    action_space = eval(['combvec(',str,')']);
    
    cum_reward = zeros(1,num_sims);
    num_moves = zeros(1,num_sims);
    trajectories = cell(1,num_sims);

    % for desired number of simulations
    for i=1:num_sims
        
       caught_flags = zeros(1,num_targets);
       
       s = randi([1,num_states^(num_agents+num_targets)]);
       
       while ~all(caught_flags)           
           
           % take agent actions, move targets
%            actions = policy(find(all(X==s)));
           action = policy(s);
           % move agents
%            states = transition_fxn{actions}*states;
           [T] = BuildTransitionMatrix2(s,action,X,num_agents,num_targets,transition_fxn,action_space,dyn);
           
           % "take action" and move target by drawing sample state from
           % transition fxn result
           s = randsample(num_states^(num_agents+num_targets),1,true,T);
           
           for j=1:num_agents
               for k=1:num_targets
                   if X(j,s) == X(num_agents + k,s)
                       caught_flags(k) = 1;
                   end
               end
           end
           
           trajectories{i}(:,end+1) = [s;action];
           
%            agent_locations = states(1:num_agents);
%            target_locations = states(num_agents+1:end);
%            
%            for j=1:length(agent_locations)
%                for k=1:length(target_locations)
%                    if agent_location(j) == target_locations(k)
%                        caught_flags(k) = 1;
%                    end
%                end
%            end
           
           num_moves(i) = num_moves(i) + 1;
%            cum_reward(i) = cum_reward(i) + utility(s);
                   
       end   
    end
end