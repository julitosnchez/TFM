function avg=ev(ptCloud,vector,center,radius)
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
double sum=0;
size=ptCloud.Count;
dists=zeros(1,size);
mod_u=sqrt(vector(1)^2+vector(2)^2+vector(3)^2);
    for j=1:size
        x=ptCloud.Location(j);
        y=ptCloud.Location(size+j);
        z=ptCloud.Location(2*size+j);

        Sx=x-center(1);
        Sy=y-center(2);
        Sz=z-center(3);

        mod_qpxu=sqrt((Sy*vector(3)-Sz*vector(2))^2+(Sz*vector(1)-Sx*vector(3))^2+(Sx*vector(2)-Sy*vector(1))^2);
        
        temp=mod_qpxu/mod_u;
        if isnan(temp)
            mod_qpxu
            x
            y
            z
            Sx
            Sy
            Sz
        end    
        dist=abs(temp-radius);
        dists(j)=dist;
        
    end
    avg=mean(dists);
end