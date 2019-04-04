% DDF exmple problem 1 - Ofer Dagan
% Version 2 - simulating measurments
% Version 3 - adding dynamic model
% column - agent number

clear all; clc; close all;
lw = 2;
fs = 16;

s = RandStream('mlfg6331_64');
% Ts = 3; % target size in [m]
DynamicModel = 2;   % 1 - dubin's car, 2 - random walk, 3 - still
stillTarget = 0;

% K = 1000;  %Time steps
% N = 100;
% ro_0 = zeros(N,1);
% ro_0([3,19,35],1) = 1/3;

Nagents = 4;   % number of agents
dx = 1;
L = 10;
xspace = dx/2:dx:L-dx/2;
% nTargetCell = Ts/dx;



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
% xTarget(40,1) = 1; % True initial location
Tloc=35; % True initial location


Pkp1k = zeros(N,N);

% Building the P matrix
% for ii=1:N
%     for jj=1:N
%         if ii>=2 && ii<=N-1 && ii==jj
%            Pkp1k(ii,jj) = 0.3; 
%         elseif (ii>=3 && ii<=N/2) && ii==jj+1  
%            Pkp1k(ii,jj) = 0.5; 
%        elseif (ii>=3 && ii>=N/2+1) && ii==jj+1  
%            Pkp1k(ii,jj) = 0.2; 
%         elseif (ii<=N-2 && ii>=N/2-1 && jj<=N-1) && ii==jj-1
%            Pkp1k(ii,jj) = 0.5;  
%          elseif (ii<=N/2-2  && jj<=N-1) && ii==jj-1
%            Pkp1k(ii,jj) = 0.2;    
%         elseif (ii==1 || ii==N) && ii==jj
%            Pkp1k(ii,jj) = 0.1; 
%         elseif ii==2 && jj==1
%            Pkp1k(ii,jj) = 0.9; 
%         elseif ii==N-1 && jj==N
%            Pkp1k(ii,jj) = 0.9;
%         else
%            Pkp1k(ii,jj) = 0;
%            
%         end
%      end
% end

% given that i'm in cell # jj the probability to go to cell #ii
for jj=1:N  
    nN = size(c{jj,1},2); % # of neighbours
    for ii=1:nN 
        Pkp1k(c{jj,1}(ii),jj) = 1/nN;
    end
end

% if stillDynamics ==1 
%     Pkp1k = Pkp1k*0;
% end

%% simulating dynamics
dt = 1; % [sec]
tspan = [0:dt:100];
xTarget = zeros(N,length(tspan));
xTarget(Tloc,1)=1;

if DynamicModel == 1  % Dubin's Car

v = 0.3;  % [m/s]
phi = -pi/18;  %[rad/sec]

L = 0.6; %[m]
theta =  pi/2; % [rad]

eta = 4.01;
zeta = 3.01;

x0Target = [zeta eta theta ]';
u=[v, phi]';

[t,xTarget] = ode45(@(t,x) Dubin(t,x,u,L),tspan,x0Target);
xTarget = xTarget';
% Turning manuever
a=figure();
a.Position = [1000 31 772 627];
plot(xTarget(1,:),xTarget(2,:))
grid on
xlabel('$$X1 \ [m]$$','Interpreter','latex')
ylabel('$$X2 \ [m]$$','Interpreter','latex')
title('True true location')
axis equal

ax1=gca;
ax1.FontSize = fs;

elseif DynamicModel == 2 % Random walk

for kk=1:length(tspan)-1
    Tloc(kk+1)=randsample(s,100,1,true,Pkp1k(:,Tloc(kk)));
    xTarget( Tloc(kk+1),kk+1) = 1;
%     xTarget(:,kk+1) = Pkp1k*xTarget(:,kk);
end

elseif DynamicModel == 3 % Still Target
    Pkp1k =eye(N);
    
end % Dynamic Model

if stillTarget==1
    Tloc = Tloc(1)*ones(1,length(tspan));
    xTarget = xTarget*0;
    xTarget(Tloc(1),:)=1;
end

%% DDF
% from whom do I fuse info?
getInfoFrom{1} = [3];
getInfoFrom{2} = [3];
getInfoFrom{3} = [1,2,4];
getInfoFrom{4} = [3];
% getInfoFrom{5} = [4];


% Sensor 1:
m{1} = [2.5, 3.5];
C{1} = [40 0; 
      0 40];
a(1) = 0.8;
  
% Sensor 2:
m{2} = [10,10];
C{2} = [40 0; 
      0 40];
a(2) = 0.2;  
% Sensor 3:
m{3} = [10,0];
C{3} = [40 0; 
      0 40];
a(3) = 0.2;  
%   Sensor 4:
m{4} = [0,0];
C{4} = [40 0; 
      0 40];
a(4) = 0.2;  
  %   Sensor 5:
m{5} = [5,0];
C{5} = [10 0; 
      0 10];
a(5) = 0.2; 
  
%%calculate different pdf values on mesh grid
for ii=1:Nagents
    % Likelihood
    % Gaussian
%     pyxVec = mvnpdf(Xgrid,m{ii},C{ii});
%     pyxCell{ii} = pyxVec/sum(pyxVec);  % pyx
%     pyxCell{ii} = min(1,pyxVec*250);  % pyx
    
    % logistic function
    r=sqrt((X1(:)-m{ii}(1)).^2+(X2(:)-m{ii}(2)).^2);
%     r=sqrt((X1(:)-m{ii}(1)).^2+(X24target(:)-m{ii}(2)).^2);

    pyxCell{ii} = 2./(1+exp(a(ii)*r));
     
  
    % Prior cell of target
    pxyVec = ones(size(x));
    pxyCell_p{ii} = pxyVec/sum(pxyVec);  % pxy

end

% Sampling meausurments:
for kk=1:size(xTarget,2)
    if DynamicModel==1
        ind1 = find(xTarget(1,kk)<=X1(1,:)+dx/2,1,'first');
        ind2 = find(xTarget(2,kk)<=X2(:,1)+dx/2,1,'first');
    % True Target Location - grid cell
        z = (ind1-1)*N+ind2;
    elseif DynamicModel==2 || DynamicModel==3
%        z =  find(xTarget(:,kk)==max(xTarget(:,kk)));
       z = Tloc(kk);
    end
    GridCellTarget(kk,:)= z ;% [z-N, z-1,z,z+1,z+N];
    
    for ii=1:Nagents
        pyx = sum(pyxCell{ii}(GridCellTarget(kk,:),1));
        yMeas(ii,kk) = randsample(s,2,1,true,[1-pyx pyx])-1;
    end
    
end

% yMeas(:,:)=1;



for ii=1:Nagents   
    for jj=1:size(getInfoFrom{ii},2)
%         Outbox{2,ii}(:,jj) = pxyCell{ii}(:,1);
        Pc_m{ii}(:,1,jj)=pxyCell_p{ii}(:,1);
    end
end
% Initializtion
Pc_p = Pc_m;

pxy_centralizedVec_0 = pxyVec/sum(pxyVec);

b1=figure();
b1.Position = [343 178 1142 813];
% 
subplot(1,2,2)
pyx1 = reshape( pyxCell{1} , size(X1));
% surf(X1,X2,pyx1,'EdgeColor','none'), 
surf(X1,X2,pyx1,'EdgeColor','none'), 
surf(X1,X2,pyx1,'EdgeColor','none','facecolor','interp'), 

% 
view(2), xlabel('X_1','FontSize',14),ylabel('X_2','FontSize',14),
colorbar
title('Sensor Likelihood function, p(y|x) - interpulated')
% 
% subplot(2,2,2)
% pyx2 = reshape( pyxCell{2} , size(X1));
% surf(X1,X2,pyx2,'EdgeColor','none'), 
% view(2), xlabel('X_1','FontSize',14),ylabel('X_2','FontSize',14),
% colorbar
% title('Sensor 2 Likelihood Function, p(y|x)')
% 
% subplot(2,2,4)
% pyx3 = reshape( pyxCell{3} , size(X1));
% surf(X1,X2,pyx3,'EdgeColor','none'), 
% view(2), xlabel('X_1','FontSize',14),ylabel('X_2','FontSize',14),
% colorbar
% title('Sensor 3 Likelihood Function, p(y|x)')
% 
% subplot(2,2,3)
% pyx4 = reshape( pyxCell{4} , size(X1));
% surf(X1,X2,pyx4,'EdgeColor','none'), 
% view(2), xlabel('X_1','FontSize',14),ylabel('X_2','FontSize',14),
% colorbar
% title('Sensor 4 Likelihood Function, p(y|x)')

% pxy1_0 = reshape(  pxyCell{1} , size(X1));
% figure(),
% surf(X1,X2,pxy1_0,'EdgeColor','none'), 
% view(2), xlabel('X_1','FontSize',14),ylabel('X_2','FontSize',14),
% colorbar
% title('Prior - p(x)')

% b1=figure();
% pyx5 = reshape( pyxCell{5} , size(X1));
% surf(X1,X2,pyx5,'EdgeColor','none'), 
% view(2), xlabel('X_1','FontSize',14),ylabel('X_2','FontSize',14),
% colorbar
% title('Sensor 5 Likelihood Function, p(y|x)')
%%
K=length(tspan);
k2print = 100;
FuseK = [1:1:K];
kk = 1;

% Centralized Fusion
pxy_centralizedVec_p(:,1) =  pxy_centralizedVec_0;

for k =1:K
    % Local information
    pxy_centralizedVec_m(:,k) = Pkp1k*pxy_centralizedVec_p(:,k);
    P_Like_cent = ones(size(pyxCell{1}));
    for ii=1:Nagents
        % Dynamic predication step
         pxyCell_m{ii}(:,k) = Pkp1k*pxyCell_p{ii}(:,k);
         
        % Bayesian measurment update
        if yMeas(ii,k)==0  % agent doesn't see target
            P_Like = 1-pyxCell{ii};
        else % agent sees target
            P_Like = pyxCell{ii};
        end
        pxyCell_p{ii}(:,k+1) = pxyCell_m{ii}(:,k).*P_Like/(pxyCell_m{ii}(:,k)'*P_Like);
        P_Like_cent = P_Like_cent.*P_Like;   
        
        
        Outbox{1,ii} = pxyCell_p{ii}(:,k+1);
        
        % Dynamic prediction of common info
        for jj=1:size(getInfoFrom{ii},2)
            Pc_m{ii}(:,k+1,jj) = Pkp1k*Pc_m{ii}(:,k,jj);
            
        end
        
        
    end
    pxy_centralizedVec_p(:,k+1) =  pxy_centralizedVec_m(:,k).*P_Like_cent;
    
    if k==FuseK(kk)
        for ii=1:Nagents
        
            Inbox{ii} = Outbox(1,getInfoFrom{ii});
            
            for jj=1:size(Inbox{ii},2)
                pxyCell_p{ii}(:,k+1) = pxyCell_p{ii}(:,k+1).*Inbox{ii}{1,jj}./Pc_m{ii}(:,k+1,jj);

                % store joint info i&j
                Pc_p{ii}(:,kk+1,jj) = Outbox{1,ii}.*Inbox{ii}{1,jj}./Pc_m{ii}(:,k+1,jj);
                Pc_p{ii}(:,kk+1,jj) = Pc_p{ii}(:,kk+1,jj)/sum(Pc_p{ii}(:,kk+1,jj));
                Pc_m{ii}(:,k+1,jj) = Pc_p{ii}(:,kk+1,jj);
            end
            % Normalize
            pxyCell_p{ii}(:,k+1) = pxyCell_p{ii}(:,k+1)/sum(pxyCell_p{ii}(:,k+1));
        end
         if ii==Nagents && kk<length(FuseK)
                kk=kk+1;
         end
    end
        
    pxy_centralizedVec_p(:,k+1)=pxy_centralizedVec_p(:,k+1)/sum(pxy_centralizedVec_p(:,k+1));    
%     pxy_centralizedVec(:,k+1) = pxy_centralizedVec(:,k);
end
    
 


% xvec = NaN*ones(N,1);
% xvec(GridCellTarget(k2print-1)) = 1;
% xyTrue=reshape((xvec), size(X1));

N1 = length(xspace);
cellT = GridCellTarget(k2print-1);

x1True = X1(cellT);
x2True = X2(cellT);

for k=1:length(GridCellTarget)
    cellT = GridCellTarget(k);
    x1TrueVec(k) = X1(cellT);
    x2TrueVec(k) = X2(cellT);
end
% x1True = xspace((floor(cellT/N1))+1);
% x2True = xspace(cellT-floor(cellT/N1)*N1);
% 
% for k=1:length(GridCellTarget)
%     cellT = GridCellTarget(k);
%     x1TrueVec(k) = xspace((floor(cellT/N1))+1);
%     x2TrueVec(k) = xspace(cellT-floor(cellT/N1)*N1);
% end

b2 = figure();
b2.Position = [343 178 1142 813];
subplot(2,2,1)
pxy1_k = reshape( pxyCell_p{1}(:,k2print) , size(X1));
surf(X1,X2,pxy1_k,'EdgeColor','none'), 
surf(X1,X2,pxy1_k,'EdgeColor','none','facecolor','interp'), 

view(2), xlabel('X_1','FontSize',14),ylabel('X_2','FontSize',14),
colorbar
title(['Sensor 1 p(x|y) at time step k = ',num2str(k2print)])
hold on
plot3(x1True,x2True,1,'rx',x1TrueVec(1),x2TrueVec(1),1,'ro',...
    'markersize',16)
plot3(x1TrueVec(1:k2print-1),x2TrueVec(1:k2print-1),ones(k2print-1,1),'r--')

% plot3(xTarget(1,:),xTarget(2,:),ones(1,K),'--r','linewidth',1)


subplot(2,2,2)
pxy2_k = reshape( pxyCell_p{2}(:,k2print) , size(X1));
surf(X1,X2,pxy2_k,'EdgeColor','none','facecolor','interp'), 
view(2), xlabel('X_1','FontSize',14),ylabel('X_2','FontSize',14),
colorbar
title(['Sensor 2 p(x|y) at time step k = ',num2str(k2print)])
hold on
plot3(x1True,x2True,1,'rx',x1TrueVec(1),x2TrueVec(1),1,'ro',...
    'markersize',16)
plot3(x1TrueVec(1:k2print-1),x2TrueVec(1:k2print-1),ones(k2print-1,1),'r--')

subplot(2,2,4)
pxy3_k = reshape( pxyCell_p{3}(:,k2print) , size(X1));
surf(X1,X2,pxy3_k,'EdgeColor','none','facecolor','interp'), 
view(2), xlabel('X_1','FontSize',14),ylabel('X_2','FontSize',14),
colorbar
title(['Sensor 3 p(x|y) at time step k = ',num2str(k2print)])
hold on
plot3(x1True,x2True,1,'rx',x1TrueVec(1),x2TrueVec(1),1,'ro',...
    'markersize',16)
plot3(x1TrueVec(1:k2print-1),x2TrueVec(1:k2print-1),ones(k2print-1,1),'r--')

subplot(2,2,3)
pxy4_k = reshape( pxyCell_p{4}(:,k2print) , size(X1));
surf(X1,X2,pxy4_k,'EdgeColor','none','facecolor','interp'), 
view(2), xlabel('X_1','FontSize',14),ylabel('X_2','FontSize',14),
colorbar
title(['Sensor 4 p(x|y) at time step k = ',num2str(k2print)])
hold on
plot3(x1True,x2True,1,'rx',x1TrueVec(1),x2TrueVec(1),1,'ro',...
    'markersize',16)
plot3(x1TrueVec(1:k2print-1),x2TrueVec(1:k2print-1),ones(k2print-1,1),'r--')
% Centralized

b3 = figure();
% b3.Position = [343 178 1142 813];
pxy_centralized_k = reshape( pxy_centralizedVec_p(:,k2print) , size(X1));
surf(X1,X2,pxy_centralized_k,'EdgeColor','none','facecolor','interp'), 
view(2), xlabel('X_1','FontSize',14),ylabel('X_2','FontSize',14),
colorbar
title(['Centalized p(x|y) at time step k = ',num2str(k2print)])
hold on
plot3(x1True,x2True,1,'rx',x1TrueVec(1),x2TrueVec(1),1,'ro',...
    'markersize',16)
plot3(x1TrueVec(1:k2print-1),x2TrueVec(1:k2print-1),ones(k2print-1,1),'r--')

function dxdt = Dubin(t,x,u,L)

theta = x(3);

v = u(1);  
phi = u(2);  

dxdt(1,1) = v*cos(theta);
dxdt(2,1) = v*sin(theta);
dxdt(3,1) = v/L*tan(phi);

end

