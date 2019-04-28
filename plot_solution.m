function [util_grid,policy_grid] = plot_solution(utility_vec,policy_vec,world_size,target_locations)

    for i=1:(world_size(1)*world_size(2))
        util_grid_loc = vec2grid(i,world_size);
        policy_grid_loc = vec2grid(i,world_size);
        
        util_quiver_vec(i,:) = util_grid_loc;
        policy_quiver_vec(i,1:2) = policy_grid_loc;
        
        util_grid(util_grid_loc(1),util_grid_loc(2)) = utility_vec(i);
        policy_grid(policy_grid_loc(1),policy_grid_loc(2)) = policy_vec(i);
        
        if policy_vec(i) == 1
            u = 0; v = 0;
        elseif policy_vec(i) == 2
            u = 0; v = 1;
        elseif policy_vec(i) == 3
            u = 0; v = -1;
        elseif policy_vec(i) == 4
            u = 1; v = 0;
        elseif policy_vec(i) == 5
            u = -1; v = 0;
        end
        
        policy_quiver_vec(i,3) = u; policy_quiver_vec(i,4) = v;
        
    end
    
    target_locs_vec = zeros(size(target_locations,1),2);
    for i=1:size(target_locations,1)
        target_locs_grid(i,:) = vec2grid(target_locations(i),world_size);
    end
    

    utility_plot_vec = reshape(utility_vec,world_size);
    policy_plot_vec = reshape(policy_vec,world_size);

    figure
    hold on; grid on;
    imagesc(util_grid)
%     for i=1:(world_size(1)*world_size(2))
%         rectangle('Position',[]
%         
%         
%     end
    quiver(policy_quiver_vec(:,2),policy_quiver_vec(:,1),policy_quiver_vec(:,3),policy_quiver_vec(:,4),'Color','k');
    titlestr = '';
    for i=1:size(target_locs_grid,1)
        rectangle('Position',[target_locs_grid(i,2)-0.5, target_locs_grid(i,1)-0.5, 1, 1],'EdgeColor','k','LineWidth',1,'LineStyle','-.')
        titlestr = strcat(titlestr,'(',num2str(target_locs_grid(i,1)),',',num2str(target_locs_grid(i,2)),'),');
    end
    
%     colormap spring
%     titlestr = strcat('Utility fxn and policy, target at (',num2str(target_loc(1)),',',num2str(target_loc(2)),')');
    title(strcat('Utility fxn and policy for targets located at: ',titlestr))
    
end

function val = in_grid(pos,world_size)
% returns if passed position is in grid

    if ((pos(1) < world_size(1)) && (pos(1) > 0)) && ...
            ((pos(2) < world_size(2)) && (pos(2) > 0))
        val = true;
    else
        val = false;
    end

end

function [vec_pos] = grid2vec(grid_coord,world_size)
% converts grid coord to vector index

    vec_pos = (grid_coord(1)-1)*world_size(2) + grid_coord(2);

end

function [grid_coord] = vec2grid(vec_pos,world_size)
% converts vec pos to grid coord

    grid_coord = [0,0];
    
    grid_coord(2) = floor((vec_pos-1)/world_size(1)) + 1;
    grid_coord(1) = mod((vec_pos-1),world_size(1)) + 1;

end