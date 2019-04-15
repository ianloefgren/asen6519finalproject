function [T] = BuildTransitionMatrix(c,a,ni,nj)
% c is a cell array that holds the cell numbers for the "Neibours" of the current state
% a is the action
% for an m x n grid: 
% ni - # of grid cells in the m direction 
% nj - # of grid cells in the n direction

N = ni*nj;
T = zeros(N,N);

for ii=1:ni
   for jj=1:nj
       s = (jj-1)*ni+ii;
       if a==1 % stay
           T(s,s)=0.8;
           T(s,c{s}(c{s}~=s)) = (1-T(s,s))/(length(c{s})-1);
       elseif a==2 % up
           if ii==ni %top row
            T(s,s)=0.8;
            T(s,c{s}(c{s}~=s)) = (1-T(s,s))/(length(c{s})-1);         
           else          
               T(s,s+1)=0.8;
               T(s,c{s}(c{s}~=(s+1))) = (1-T(s,s+1))/(length(c{s})-1);
           end
       elseif a==3 % down
           if ii==1 %bottom row
            T(s,s)=0.8;
            T(s,c{s}(c{s}~=s)) = (1-T(s,s))/(length(c{s})-1);         
           else          
               T(s,s-1)=0.8;
               T(s,c{s}(c{s}~=(s-1))) = (1-T(s,s-1))/(length(c{s})-1);
           end       
       elseif a==4 % right
           if jj==nj %right column
            T(s,s)=0.8;
            T(s,c{s}(c{s}~=s)) = (1-T(s,s))/(length(c{s})-1);         
           else          
               T(s,s+ni)=0.8;
               T(s,c{s}(c{s}~=(s+ni))) = (1-T(s,s+ni))/(length(c{s})-1);
           end          
       elseif a==5 % left
            if jj==1 %left column
            T(s,s)=0.8;
            T(s,c{s}(c{s}~=s)) = (1-T(s,s))/(length(c{s})-1);         
           else          
               T(s,s-ni)=0.8;
               T(s,c{s}(c{s}~=(s-ni))) = (1-T(s,s-ni))/(length(c{s})-1);
           end                     
             
       end
   end
    
    
    
    
end














end