function [min_d,Px,Py] = FindNarrowAreas(ObsX,ObsY)
%%
[len,~] = size(ObsX);
min_d = 0;
min_d0 = inf;
for ii = 1:len
    P1.x = ObsX(ii,:);
    P1.y = ObsY(ii,:);
    for zz = 1:len
        if ii == zz
            continue
        end
        P2.x = ObsX(zz,:);
        P2.y = ObsY(zz,:);
        [min_d,px,py] = min_dist_between_two_polygons(P1,P2,1);
        if min_d<=min_d0
            min_d0=min_d;
            Px = px;
            Py = py;
        end
    end
end

