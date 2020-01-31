function [pop]=initiate_pop(min,max,NP,D)
%--------------------------------------------------------------------------
%   Function: initiate_pop
%--------------------------------------------------------------------------
% -> Description: Initial population generation. A random population of NP
% candiates, each one with D chromosomes, randomly generated
%--------------------------------------------------------------------------
% -> Inputs:
%       -min: Minimum coordinate values in the cloud.
%       -max: Maxmum coordinate values in the cloud.
%       -NP: population size.
%       -D:Number of chromosomes
%--------------------------------------------------------------------------

poppre=zeros(NP,D);
cost=zeros(NP,1);
for i=1:NP
   poppre(i,:) = min + rand(1,D).*(max- min);
end
for i=1:NP
    for j=1:D
        if poppre(i,j)<min(j), poppre(i,j)=min(j);end
        if poppre(i,j)>max(j), poppre(i,j)=max(j);end
    end
end
pop=[cost poppre];
end