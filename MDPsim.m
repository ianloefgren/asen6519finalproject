% MDP simulator function
%
% Runs simulations fkfkfk
%

function [trajectories, avg_num_moves, avg_cum_reward] = MDPsim(utility,policy,transition_fxn,X,num_sims,num_agents,num_targets,num_states,dyn)

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
           
           % check to see if any target has been caught by agents
           for j=1:num_agents
               for k=1:num_targets
                   if X(j,s) == X(num_agents + k,s)
                       caught_flags(k) = 1;
                   end
               end
           end
           
           % record action state pair, reward, and move count
           trajectories{i}(:,end+1) = [s;action];
           cum_reward(i) = cum_reward(i) + utility(s);
           num_moves(i) = num_moves(i) + 1;
                   
       end   
    end
    
    % compute average cumulative reward and number of moves
    avg_num_moves = sum(num_moves)/num_sims;
    avg_cum_reward = sum(cum_reward)/num_sims;
    
end