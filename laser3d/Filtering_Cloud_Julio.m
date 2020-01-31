% First, remove noise of Axis
np1 = removerows(nube1.Location,nube1.Location(:,1) > -2);
np1 = removerows(np1,np1(:,2) > -0.5);
np1 = removerows(np1,np1(:,2) < -3);

np1 = pointCloud(np1);

np2 = removerows(nube3.Location,nube3.Location(:,1) > -2);
np2 = removerows(np2,np2(:,2) > -0.5);
np2 = removerows(np2,np2(:,2) < -3);

np2 = pointCloud(np2);

% Then, lets fit planes with RANSAC
plane_n1 = pcfitplane(np1,0.0017);
plane_n2 = pcfitplane(np2,0.0017);

% Calculating the angle between the two planes
coseno=abs(dot(plane_n1.Normal,plane_n2.Normal)/(norm(plane_n1.Normal)*norm(plane_n2.Normal)));

% Data we are interested in
angulo=acos(coseno);
angulo_deg = rad2deg(angulo);
vector_giro = cross(plane_n1.Normal,plane_n2.Normal);

