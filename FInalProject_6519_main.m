% 6519 - Aerospace Algorithms Final Project

% column - agent number

%clear all;% clc; close all;
lw = 2;
fs = 16;
plotFlag = 0;
AlgoFlag = 1;   % 1 - QMDP, 2 - FIB
seed = RandStream('mlfg6331_64');
runMDPsim = 1;
runPOMDPsim = 1;

if ~exist('Pol','var')
    DynamicModel = 1;   % 1 - random walk, 2 - still target

    nAgents = 1;   % number of agents
    nTargets = 1;
end

L = 7;
dx = 1;
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
cTarget = whoRmyNeighbours(ni,nj,1);


Tloc=[1]'; % 50]; % Target True initial location
Aloc=[16]'; % 50]; % Agent True initial location

xTrue = [Tloc ; Aloc];

Pkp1k = zeros(N,N);

% given that i'm in cell # jj the probability to go to cell #ii
if DynamicModel == 1   % Random walk
    for jj=1:N  
        nN = size(cTarget{jj,1},2); % # of neighbours
        for ii=1:nN 
            Pkp1k(cTarget{jj,1}(ii),jj) = 1/nN;
        end
    end
elseif DynamicModel == 2 % Still Target
        Pkp1k =eye(N);

end % Dynamic Model
%% Likelihood model for each grid point
% pyxCell - each column corresponds to cell number
% position is defined by the cell center
a = 0.6;
for ii=1:N
    iN = floor((ii-1)/nj)+1;
    jN = ii-(iN-1)*ni;
    
    xi = xspace(iN);
    yi = xspace(jN);
    cellCenter(:,ii) = [xi ; yi];
    
    r=sqrt((X1(:)-xi).^2+(X2(:)-yi).^2);
    pyxCell(:,ii) = 2./(1+exp(a*r));

end

%% State Transition Matrix
% P(s'|s,a) -->  P{a}(s,s')
% a = 1 - stay
% a = 2 - up, a = 3 - down
% a = 4 - right, a = 5 - left

% Rows sum up to 1
cAgent = whoRmyNeighbours(ni,nj,2);

% for a single agent:
for a = 1:5
    P{a} = BuildTransitionMatrix(cAgent,a,ni,nj);
end 


%% MDP solution

save_str = strcat(num2str(nAgents),'agents_',num2str(nTargets),'targets_',num2str(L),'x',num2str(L),'_dyn',num2str(DynamicModel),'.mat');
if ~exist('Pol','var')

    [Pol,Val,X,A,R] = MDP_FinalProject(P,nAgents,nTargets,N,DynamicModel,Pkp1k,L);

    save(save_str,'Pol','Val','X','A','R','nAgents','nTargets','DynamicModel')
end

%%
target_loc = [42]';
% =======
%%
target_loc = [42]';

if nTargets==1
    ind = find(X(1,:) == target_loc(1));
else
    [row,ind]= find(sum(X(1:nTargets,:)==target_loc)==nTargets);
end

polSlice = Pol(ind);
utilSlice = Val(ind);
X2=flipud(X);



%%
%  [trajectories,avg_num_moves,avg_cum_reward] = MDPsim(Val,Pol,P,X,1000,nAgents,nTargets,N,DynamicModel);
    

%%
if DynamicModel == 2
    plot_solution(utilSlice,polSlice,[L,L],target_loc);
end

%% MDP Simulations

% Run num_sims number of monte carlo simulations using the computed policy
% and a greedy policy, and compute performance metrics.
if runMDPsim
num_sims = 1000;

[t, anm, acr, gt, ganm, gacr] = MDPsim(Val,Pol,P,Pkp1k,X,num_sims,nAgents,...
                                        nTargets,N,DynamicModel,[L,L]);
    
% reassign output to descriptive variable names because matlab's return
% syntax is weird / I don't know how to do it properly
trajectories = t;
avg_num_moves = anm;
avg_cum_reward = acr;
greedy_trajectories = gt;
greedy_avg_num_moves = ganm;
greedy_avg_cum_reward = gacr;

% print summary of performance
fprintf('\nMDP performance: %i Monte Carlo simulations: \n',num_sims)
fprintf('================================================\n')
fprintf('\t\t\t MDP \t\t Greedy\n')
fprintf('------------------------------------------------\n')
fprintf('Avg moves \t| \t %.2f \t\t %.2f\n',avg_num_moves,greedy_avg_num_moves)
fprintf('Avg reward \t| \t %.2f \t %.2f\n',avg_cum_reward,greedy_avg_cum_reward)
                                                            
end                                                           
%% POMDP
%% Build likelihood model to generate measurements 
% PojX is the joint likelihood of detecting a target in cell jj, given the state
% of the agents is sA

v = 1:N;
str = 'v';
for ii=1:nAgents-1
    str = [str,',v'];
end
S_A=eval(['combvec(',str,')']);

% pmx = eye(size(pyxCell));  % agent likelihood of own state m
pmx = zeros(N,N);
c=cAgent;
for ii=1:ni
   for jj=1:nj
       s = (jj-1)*ni+ii;     
               pmx(s,s)=0.95;
               pmx(s,c{s}(c{s}~=s)) = (1-pmx(s,s))/(length(c{s})-1);         

   end
end

pOjX = ones(N,size(S_A,2));
for sA=1:size(S_A,2)
    for jj=1:N
        for ii=1:nAgents  
            P_Like =pyxCell(jj,S_A(ii,sA));
            pOjX(jj,sA) = pOjX(jj,sA)*P_Like;
        end
    end
end


% [Q] = POMDP_FinalProject(P,nAgents,nTargets,X,DynamicModel,Val,A,R,pOjX,pmx,AlgoFlag,S_A,Pkp1k,xTrue,seed,L,save_str);

%% POMDP Simulations

% Run num_sims number of monte carlo simulations using the computed policy
% and a greedy policy, and compute performance metrics.
if runPOMDPsim
num_sims = 1000;

avg_num_moves = 0;
pomdp_trajectories = cell(1,num_sims);
beliefs = cell(1,num_sims);

for i=1:num_sims

    if ~exist('Q','var')
        Q = false;
    end
    [~,trajectory,belief,num_moves,~] = POMDP_FinalProject(P,nAgents,nTargets,X,DynamicModel,Val,A,R,pOjX,pmx,AlgoFlag,S_A,Pkp1k,xTrue,seed,L,save_str,Q);

    pomdp_trajectories{i} = trajectory{1};
    beliefs{i} = belief{1};
    avg_num_moves = avg_num_moves + num_moves;

end
    
avg_num_moves = avg_num_moves / num_sims;

% reassign output to descriptive variable names because matlab's return
% syntax is weird / I don't know how to do it properly
% trajectories = t;
% avg_num_moves = anm;
% avg_cum_reward = acr;
% greedy_trajectories = gt;
greedy_avg_num_moves = 0;
% greedy_avg_cum_reward = gacr;

% print summary of performance
fprintf('\nPOMDP performance: %i Monte Carlo simulations: \n',num_sims)
fprintf('================================================\n')
fprintf('\t\t\t POMDP \t\t Greedy\n')
fprintf('------------------------------------------------\n')
fprintf('Avg moves \t| \t %.2f \t\t %.2f\n',avg_num_moves,greedy_avg_num_moves)
% fprintf('Avg reward \t| \t %.2f \t %.2f\n',avg_cum_reward,greedy_avg_cum_reward)

end