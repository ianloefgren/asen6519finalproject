% 6519 - Aerospace Algorithms Final Project

% column - agent number

clear all; clc; close all;
lw = 2;
fs = 16;
plotFlag = 1;

s = RandStream('mlfg6331_64');

DynamicModel = 1;   % 1 - random walk, 2 - still target


nAgents = 3;   % number of agents
nTargets =2;
dx = 1;
L = 10;
xspace = dx/2:dx:L-dx/2;


[X1,X2] = meshgrid(xspace);
X24target = X2;
for jj=2:2:size(X2,2)
    X24target(:,jj) = flipud(X2(:,jj));
end
Xgrid = [X1(:),X2(:)];
x = [1:1:length(Xgrid)]';

ni = size(X1,1);
nj = size(X1,2);
N = length(x);
[c] = whoRmyNeighbours(ni,nj);

Tloc=[5 50]; % True initial location


Pkp1k = zeros(N,N);

% given that i'm in cell # jj the probability to go to cell #ii
for jj=1:N  
    nN = size(c{jj,1},2); % # of neighbours
    for ii=1:nN 
        Pkp1k(c{jj,1}(ii),jj) = 1/nN;
    end
end

%% Likelihood model for each grid point
% pyxCell - each column corresponds to cell number
% position is defined by the cell center
a = 0.9;
for ii=1:N
    iN = floor((ii-1)/nj)+1;
    jN = ii-(iN-1)*ni;
    
    xi = xspace(iN);
    yi = xspace(jN);
    cellCenter(:,ii) = [xi ; yi];
    
    r=sqrt((X1(:)-xi).^2+(X2(:)-yi).^2);
    pyxCell(:,ii) = 2./(1+exp(a*r));

end

% visualization
if plotFlag
    b1=figure();
    b1.Position = [343 178 1142 813];
    % 
    pyx1 = reshape( pyxCell(:,100) , size(X1));
    % surf(X1,X2,pyx1,'EdgeColor','none'), 
    surf(X1,X2,pyx1,'EdgeColor','none'), 
    surf(X1,X2,pyx1,'EdgeColor','none','facecolor','interp'), 
    view(2), xlabel('X_1','FontSize',14),ylabel('X_2','FontSize',14),
    colorbar
    title('Sensor Likelihood function, p(y|x) - interpulated')
end

    
%% Simulate targets dynamics and produce measurements    
% xTarget{jj} is a matrix representation of target jj position
% Position is indicated by "1" in the cell number corresponding to target
% location.
% Each column is a time step

dt = 1; % [sec]
tspan = 0:dt:100;

% Targets dynamics

for jj=1:nTargets
    xTarget{jj} = zeros(N,length(tspan));
    xTarget{jj}(Tloc(1,jj),1)=1;
    if DynamicModel == 1 % Random walk

    for kk=1:length(tspan)-1
        Tloc(kk+1,jj)=randsample(s,100,1,true,Pkp1k(:,Tloc(kk,jj)));
        xTarget{jj}( Tloc(kk+1,jj),kk+1) = 1;
    end

    elseif DynamicModel == 2 % Still Target
        Pkp1k =eye(N);

    end % Dynamic Model

    if DynamicModel == 2 % if target is still
        Tloc(:,jj) = Tloc(1,jj)*ones(1,length(tspan));
        xTarget = xTarget*0;
        xTarget{jj}(Tloc(1,jj),:)=1;
    end
end


% Sampling meausurments:
% yMeas{ii,jj}(nn,kk) is the measurement taken by agent ii of target jj
% when the agent is in cell number nn and the time step is kk
for jj=1:nTargets
    for kk=1:size(tspan,2)

        GridCellTarget(kk,jj)= Tloc(kk,jj);% [z-N, z-1,z,z+1,z+N];

        for ii=1:nAgents
            for nn=1:N
                pyx = pyxCell(GridCellTarget(kk,jj),nn);
                yMeas{ii,jj}(nn,kk) = randsample(s,2,1,true,[1-pyx pyx])-1;
            end
        end

    end
end

if plotFlag    % plotting target trajectory
    k2print = 101;
    N1 = length(xspace);
    cellT = GridCellTarget(k2print-1,1);

    x1True = X1(cellT);
    x2True = X2(cellT);

    for k=1:length(GridCellTarget)
        cellT = GridCellTarget(k,1);
        x1TrueVec(k) = X1(cellT);
        x2TrueVec(k) = X2(cellT);
    end



    b2 = figure();
    b2.Position = [343 178 1142 813];

    plot(x1True,x2True,'rx',x1TrueVec(1),x2TrueVec(1),'ro',...
        'markersize',16)
    hold on
    plot(x1TrueVec(1:k2print-1),x2TrueVec(1:k2print-1),'r--')
    grid on
end
