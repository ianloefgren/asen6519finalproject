% ASEN 6519 Final Project
%
% POMDP belief and trajectory visualization
%
% Visualizes agents belief and agent and target true locations over a
% trajectory.
%
% Inputs:
%   state_trajectory -- a trajectory of states, as output from POMDP_FinalProject
%   belief_trajectory -- a trajectory of beliefs, as above
%   X -- state enumeration vector
%   A -- action enumeration vector
%   num_agents
%   num_targets
%   world_size -- 2x1 vector of grid size ([L,L] usually)
%   Obs - observations taken in the simulation
%   record -- flag to record trajectories to video, default is false

function POMDPViz(state_trajectory,belief_trajectory,X,A,num_agents,num_targets,world_size,Obs,record)

    % make record flag optional argument
    if nargin < 9
        record = false;
    end
                         
    % decompose state into induvidual locations in grid world
    world_locations = zeros(num_agents+num_targets,size(state_trajectory,2));
    % decompose actions into induvidual actions
    actions = zeros(num_agents,size(state_trajectory,2));
    
    % get grid cell locations for each robot from state
    for i=1:size(state_trajectory,2)
        world_locations(:,i) = X(:,state_trajectory(1,i));
        actions(:,i) = A(:,state_trajectory(2,i));
    end
    
    figure
    set(gcf, 'Position', get(0, 'Screensize'));
    for t=1:size(state_trajectory,2)
        clf;
    
        % FOR a timestep in belief trajectory
%         t = 1;
        b = belief_trajectory(:,t);

        % marginalize over agent states (assuming perfect agent state knowledge
        b_targets = zeros(num_targets,size(X,2));
    %     for i=1:num_agents
    %         b_targets

        % marginalize over states to get probability of a target being in each
        % grid cell

        % FOR each target
        % marginalize over all but one target state
        target_marg_prob = zeros(world_size(1)*world_size(2),1);
        for i=1:size(target_marg_prob,1)
            target_marg_prob(i) = sum(b(X(1,:)==i));
        end
        
        indv_agent_marg_prob = zeros(num_agents,world_size(1)*world_size(2));
        % marginalize over agent states
        for idx = 1:num_agents
%             indv_agent_marg_prob(idx,:) = zeros(world_size(1)*world_size(2),1);
            for i=1:size(indv_agent_marg_prob,2)
                indv_agent_marg_prob(idx,i) = sum(b(X(num_targets+idx,:)==i));
            end
        end
        agent_marg_prob = sum(indv_agent_marg_prob,1);
        agent_marg_prob = agent_marg_prob ./ sum(agent_marg_prob);

        % convert vector belief into grid belief
        target_belief_plot_mat = zeros(world_size(1));
        for i=1:length(target_marg_prob)

            % get grid coords from position in loop
            grid_coord = vec2grid(i,world_size);

            % populate belief grid representation using grid coords and prob
            target_belief_plot_mat(grid_coord(1),grid_coord(2)) = target_marg_prob(i);
        end
        
        % convert vector belief into grid belief
        agent_belief_plot_mat = zeros(world_size(1));
        for i=1:length(agent_marg_prob)

            % get grid coords from position in loop
            grid_coord = vec2grid(i,world_size);

            % populate belief grid representation using grid coords and prob
            agent_belief_plot_mat(grid_coord(1),grid_coord(2)) = agent_marg_prob(i);
        end

        % plot marginzalized belief as heatmap
    %     figure
        subplot(1,2,1)
        hold on;
        imagesc(target_belief_plot_mat)
%         s1 = pcolor(target_belief_plot_mat);
%         s1.FaceColor = 'interp';
%         s1.EdgeColor = 'none';
        colorbar;
        
%         xspace = 1:1:world_size(1);
%         [X1,X2] = meshgrid(xspace);
%         surf(fliplr(X1),flipud(X2),target_belief_plot_mat,'EdgeColor','none','facecolor','interp')
%         colorbar;

        % find corresponding true target and agent positions from state
        % trajectory

        % plot true positions
        for i=1:size(world_locations,1)
            if i > num_targets
                marker = 'ro';
            else
                marker = 'rx';
            end

            grid_coord = vec2grid(world_locations(i,t),world_size);
            plot(grid_coord(2),grid_coord(1),marker,'MarkerSize',16)
        end
        
        subplot(1,2,2)
        hold on;
        imagesc(agent_belief_plot_mat)
%         s2 = pcolor(agent_belief_plot_mat);
%         s2.FaceColor = 'interp';
%         s2.EdgeColor = 'none';
        colorbar;
        
%         xspace = 1:1:world_size(1);
%         [X1,X2] = meshgrid(xspace);
%         surf(fliplr(X1),flipud(X2),agent_belief_plot_mat,'EdgeColor','none','facecolor','interp')
%         colorbar;

        % find corresponding true target and agent positions from state
        % trajectory

        % plot true positions
        for i=1:size(world_locations,1)
            if i > num_targets
                marker = 'ro';
            else
                marker = 'rx';
            end

            grid_coord = vec2grid(world_locations(i,t),world_size);
            plot(grid_coord(2),grid_coord(1),marker,'MarkerSize',16)
        end
        
        titleStr='Observation: ';
        if t<size(state_trajectory,2)
            for j=1:num_targets
               
                titleStr=[titleStr,'T_',num2str(j),'=',num2str(Obs(j,t))]; 
            end
            for i=1:num_agents
               titleStr=[titleStr,' ; A_',num2str(i),'=',num2str(Obs(i+num_targets,t))]; 
            end
            title(titleStr)
        end
        if record
            drawnow;
            pause(0.5);
            mov(:,t) = getframe(gcf);
        else
            input('press enter to display next timestep')
        end
        
        
    end
    
    disp(actions)
    
    if record
        mov = repelem(mov,1,30*ones(1,size(mov,2)));
        mov_title = strcat('belief_',num2str(num_agents),'agents_',num2str(num_targets),'targets_',num2str(world_size(1)),'x',num2str(world_size(2)));
        v = VideoWriter(mov_title,'Motion JPEG AVI');
        open(v);
        writeVideo(v,mov);
        close(v);
    end
    
%     figure
%     hold on;
%     
%     % create grid cell 
%     for i=1:world_size(1)
%         for j=1:world_size(2)
%             rectangle('Position',[i-0.5,j-0.5,1,1],'EdgeColor','k')
%         end
%     end
%     rectangle('Position',[0.5,0.5,world_size(1),world_size(2)],'EdgeColor','k','LineWidth',2)
                
                
                
                
                
end

function [grid_coord] = vec2grid(vec_pos,world_size)
% converts vec pos to grid coord

    grid_coord = [0,0];
    
    grid_coord(2) = floor((vec_pos-1)/world_size(1)) + 1;
    grid_coord(1) = mod((vec_pos-1),world_size(1)) + 1;

end
