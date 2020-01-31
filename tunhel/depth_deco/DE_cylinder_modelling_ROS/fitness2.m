function [error]=fitness2(ptCloud,trial)
%--------------------------------------------------------------------------
%   Function: fitness
% -> Description: fitness function that is optimized by the DE-based Global
% localization filter. 
%--------------------------------------------------------------------------
% -> Inputs:
%       -ptCloud: Hole point cloud.
% -> Output: 
%       -error: fitness value.
%--------------------------------------------------------------------------

sum_desv=0;
size=ptCloud.Count;
dists=zeros(1,size);


    for j=1:size
        x=ptCloud.Location(j);
        y=ptCloud.Location(size+j);
        z=ptCloud.Location(2*size+j);

        Sx=x-trial(1);
        Sy=y-trial(2);
        Sz=z-trial(3);

        mod_qpxu=sqrt((Sy*trial(6)-Sz*trial(5))^2+(Sz*trial(4)-Sx*trial(6))^2+(Sx*trial(5)-Sy*trial(4))^2);
        mod_u=sqrt(trial(4)^2+trial(5)^2+trial(6)^2);
        dist_rect=mod_qpxu/mod_u;
        dists(j)=dist_rect-trial(7);
     end

   

    for j=1:size
        desv=dists(j)^2;
        sum_desv=sum_desv + desv;
    end

    desvest=sqrt(sum_desv/size);
    error=desvest;
end

