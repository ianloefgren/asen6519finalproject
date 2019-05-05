% MDP simulator function
%
% Runs simulations fkfkfk
%

function [trajectories, avg_num_moves, avg_cum_reward, greedy_trajectories, greedy_avg_num_moves, greedy_avg_cum_reward] = MDPsim(utility,policy,transition_fxn,...
                                                                target_transition_fxn,X,num_sims,num_agents,num_targets,...
                                                                num_states,dyn,world_size)

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

    greedy_cum_reward = zeros(1,num_sims);
    greedy_num_moves = zeros(1,num_sims);
    greedy_trajectories = cell(1,num_sims);

    
    % for desired number of simulations
    for i=1:num_sims
        
       caught_flags = zeros(1,num_targets);
       greedy_caught_flags = zeros(1,num_targets);
       
       s = randi([1,num_states^(num_agents+num_targets)]);
       greedy_s = randi([1,num_states^(num_agents+num_targets)]);
       
       while ~all(caught_flags)           
           
           % take agent actions, move targets
%            actions = policy(find(all(X==s)));
           action = policy(s);
           % move agents
%            states = transition_fxn{actions}*states;

           % generate transitions for agents
           T= BuildTransitionMatrix2(s,X,num_agents,num_targets,transition_fxn,action_space,dyn,target_transition_fxn);
           T = T(action,:);
           
           % get new state
           s = randsample(num_states^(num_agents+num_targets),1,true,T);
            
           for j=1:num_agents
               for k=1:num_targets
                   if X(num_targets+j,s) == X(k,s)
                       caught_flags(k) = 1;
                   end
               end
           end
           
           % record action state pair, reward, and move count
           trajectories{i}(:,end+1) = [s;action;caught_flags'];
           cum_reward(i) = cum_reward(i) + utility(s);
           num_moves(i) = num_moves(i) + 1;
       end
            
       while ~all(greedy_caught_flags)
           
           % take agent actions, move targets
%            actions = policy(find(all(X==s)));
           greedy_a = greedy_action(greedy_s,X,num_agents,num_targets,world_size);
           % move agents
%            states = transition_fxn{actions}*states;

           % generate transitions for agents
           greedy_T = BuildTransitionMatrix2(greedy_s,X,num_agents,num_targets,transition_fxn,action_space,dyn,target_transition_fxn);
           greedy_T = greedy_T(greedy_a,:);

           % get new state
           greedy_s = randsample(num_states^(num_agents+num_targets),1,true,greedy_T);
           
           % check to see if any target has been caught by agents
           for j=1:num_agents
               for k=1:num_targets
%                    if X(j,greedy_s) == X(num_agents+k,greedy_s)
                   if X(num_targets+j,greedy_s) == X(k,greedy_s)
                       greedy_caught_flags(k) = 1;
                   end
               end
           end
           
           % record action state pair, reward, and move count
           greedy_trajectories{i}(:,end+1) = [greedy_s;greedy_a;greedy_caught_flags'];
           greedy_cum_reward(i) = greedy_cum_reward(i) + utility(greedy_s);
           greedy_num_moves(i) = greedy_num_moves(i) + 1;        
       end    
    end
    
    % compute average cumulative reward and number of moves
    avg_num_moves = sum(num_moves)/num_sims;
    avg_cum_reward = sum(cum_reward)/num_sims;
    
    greedy_avg_num_moves = sum(greedy_num_moves)/num_sims;
    greedy_avg_cum_reward = sum(greedy_cum_reward)/num_sims;
    
end

function [action] = greedy_action(s,X,num_agents,num_targets,world_size)
    % Computes greedy actions for all agents, where greedy action is to reduce
    % the Manhattan distance between an agent and the closest target.
    
    % convert state representation to induvidual representations
    % decompose state into induvidual locations in grid world
    world_location = X(:,s);
    
    % compute grid coordinates from world locations
    grid_locations = zeros(length(world_location),2);
    for i=1:length(world_location)
        grid_locations(i,:) = vec2grid(world_location(i),world_size);
    end
    
    % compute Manhattan distance from each agent to each target using grid
    % coordinates
    action = zeros(num_agents,1);
    for i=1:num_agents
        
        min_distance = 999999999;
        
        for j=1:num_targets
            for a=1:5
                
                if a == 1
                    act = [0,0];
                elseif a == 2
                    act = [0,1];
                elseif a == 3
                    act = [0,-1];
                elseif a == 4
                    act = [1,0];
                elseif a == 5
                    act = [-1,0];
                end
          
                if sum(grid_locations(j,:) - (grid_locations(num_targets+i,:)+act)) < min_distance
                    min_distance = abs(sum(grid_locations(j,:) - (grid_locations(num_targets+i,:)+act)));
                    action(i) = a;
                end
            end
        end
    end
    
    % convert action tuple to vector location in action space
    if num_agents == 2
        action = grid2vec(action,[5,5]);
    end
    
end

function [grid_coord] = vec2grid(vec_pos,world_size)
% converts vec pos to grid coord

    grid_coord = [0,0];
    
    grid_coord(2) = floor((vec_pos-1)/world_size(1)) + 1;
    grid_coord(1) = mod((vec_pos-1),world_size(1)) + 1;

end

function [vec_pos] = grid2vec(grid_coord,world_size)
% converts grid coord to vector index

    vec_pos = (grid_coord(1)-1)*world_size(2) + grid_coord(2);

end