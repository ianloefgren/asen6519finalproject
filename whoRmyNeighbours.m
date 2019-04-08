function [c] = whoRmyNeighbours(ni,nj,Type)
% This function gets the number of grid points in each direction,
% and returns a cell array (ni*nj) x 1 of the neighbours of each cell
% for an m x n grid: 
% ni - # of grid cells in the m direction 
% nj - # of grid cells in the n direction
% Type defines the type of nerighbours (and then usually the dynamics)
% If Type=1 
% Neighbours could be the next cell horizontally, vertically, diagonally
% and the same cell

% If Type=2 
% Neighbours could be the next cell horizontally, vertically, and the same cell

% ij is the current cel number

if Type==1
    for ii=1:ni
        for jj=1:nj
            ij = (jj-1)*ni+ii;
            if ii==1 && jj==1
                c{ij,1} = [ij, 2, ni+1, ni+2];
            elseif ii==ni && jj==1
                c{ij,1} = [ij,ni-1, 2*ni-1, 2*ni];
            elseif jj==1
                c{ij,1} = [ij,ii-1, ii+1, ii+ni-1, ii+ni, ii+ni+1];
            elseif ii==1 && jj~=nj
                c{ij,1} = [ij,ij-ni, ij-ni+1,ij+1, ij+ni, ij+ni+1];
            elseif ii==1  % jj=nj
                c{ij,1} = [ij,ij-ni, ij-ni+1, ij+1];
            elseif ii==ni && jj~=nj
                c{ij,1} = [ij,ij-ni-1, ij-ni, ij-1, ij+ni-1, ij+ni];
            elseif ii==ni % jj=nj
                c{ij,1} = [ij,ij-ni-1,ij-ni, ij-1];
            elseif jj==nj
                c{ij,1} = [ij,ij-ni-1, ij-ni, ij-ni+1, ij-1,ij+1];
            else
                c{ij,1} = [ij,ij-ni-1, ij-ni, ij-ni+1, ij-1,ij+1 , ij+ni-1, ij+ni, ij+ni+1];

            end
        end
    end

elseif Type==2
     for ii=1:ni
        for jj=1:nj
            ij = (jj-1)*ni+ii;
            if ii==1 && jj==1
                c{ij,1} = [ij, 2, ni+1];
            elseif ii==ni && jj==1
                c{ij,1} = [ij,ni-1,  2*ni];
            elseif jj==1
                c{ij,1} = [ij,ii-1, ii+1,  ii+ni, ];
            elseif ii==1 && jj~=nj
                c{ij,1} = [ij,ij-ni, ij+1, ij+ni, ];
            elseif ii==1  % jj=nj
                c{ij,1} = [ij,ij-ni, ij+1];
            elseif ii==ni && jj~=nj
                c{ij,1} = [ij, ij-ni, ij-1,  ij+ni];
            elseif ii==ni % jj=nj
                c{ij,1} = [ij,ij-ni, ij-1];
            elseif jj==nj
                c{ij,1} = [ij, ij-ni,ij-1,ij+1];
            else
                c{ij,1} = [ij, ij-ni,  ij-1,ij+1 , ij+ni];

            end
        end
    end

end