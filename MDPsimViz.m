% MDP simulation visualization
%
% Visualization for MC sims of agents following policy to catch dynanmics
% targets.
%
% Inputs:
%   
%   trajectory -- trajectory to be plotted, list of state action pairs
%   X -- enumeration of possible states to use find fxn on
%   num_agents -- number of agents in sim
%   num_targets -- number of targets in sim
%   world_size -- size of grid world [x,y]
%
% Returns:
%
%   none
%

function MDPsimViz(trajectory,X,num_agents,num_targets,world_size)

    color_wheel = [0    0.4470    0.7410;
    0.8500    0.3250    0.0980;
    0.9290    0.6940    0.1250;
    0.4940    0.1840    0.5560;
    0.4660    0.6740    0.1880;
    0.3010    0.7450    0.9330;
    0.6350    0.0780    0.1840];


    markers = ['x','o','d'];

    % decompose state into induvidual locations in grid world
    world_locations = zeros(num_agents+num_targets,size(trajectory,2));
    % decompose actions into induvidual actions
    actions = zeros(num_agents,size(trajectory,2));
    
    for i=1:size(trajectory,2)
        world_locations(:,i) = X(:,trajectory(1,i));
%         actions(:,i) = AX(:,s);
    end
    
    figure
    hold on;
    
    % create grid cell 
    for i=1:world_size(1)
        for j=1:world_size(2)
            rectangle('Position',[i-0.5,j-0.5,1,1],'EdgeColor','k')
        end
    end
    rectangle('Position',[0.5,0.5,world_size(1),world_size(2)],'EdgeColor','k','LineWidth',2)
    
    % plot trajectories
    for i=1:size(world_locations,2)
        plot_flag = true;
        for j=1:size(world_locations,1)
            
            grid_coord = vec2grid(world_locations(j,i),world_size);
            marker_color = color_wheel(j,:);
            
            if i==1
%                 marker_color = 'k';
                next_coord = vec2grid(world_locations(j,i+1),world_size);
                plot(grid_coord(2),grid_coord(1),markers(j),'Color','k','MarkerSize',10)
            elseif i==size(world_locations,2)
                marker_color = 'k';
                next_coord = grid_coord;
            else
                next_coord = vec2grid(world_locations(j,i+1),world_size);
                marker_color = color_wheel(j,:);
            end
            
            
            if j>num_agents
                if trajectory(1+j,i)
                    plot_flag = false;
                end
            end
            
            if plot_flag
                quiver(grid_coord(2),grid_coord(1),next_coord(2)-grid_coord(2),next_coord(1)-grid_coord(1),'Color',marker_color)
                text(grid_coord(2)+0.5*(next_coord(2)-grid_coord(2)),grid_coord(1)+0.5*(next_coord(1)-grid_coord(1)),strcat('t=',num2str(i)))
            end
%             plot(grid_coord(1),grid_coord(2),markers(j),'Color',marker_color,'MarkerSize',10)
        end
        input('press enter to display next timestep')
    end
    
%     legend('target 1','target 2','agent 1','Location','NorthEastOutside')
    legend('agent 1','target 1','target 2','Location','NorthEastOutside')
end


function [grid_coord] = vec2grid(vec_pos,world_size)
% converts vec pos to grid coord

    grid_coord = [0,0];
    
    grid_coord(2) = floor((vec_pos-1)/world_size(1)) + 1;
    grid_coord(1) = mod((vec_pos-1),world_size(1)) + 1;

end