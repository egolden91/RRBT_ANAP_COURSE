function [MinDist] = FindObsEdge(SamplePoint,ObsX,ObsY,EPS)
persistent  edge 
MinDist = inf;
[len,~] = size(ObsX);
for ii = 1:len
    for zz = 1:length(ObsX)
        if dist(SamplePoint,[ObsX(ii,zz) ObsY(ii,zz)])<EPS 
            edge = [edge; [ObsX(ii,zz) ObsY(ii,zz) ii]];
        end
    end
end

%We assume the lidar can find in which direction the edge is pointing 
[s,~] = size(edge);
if s > 2  
    for ii = 1:s
        for jj = 1:s
            if edge(ii,3) ~= edge(jj,3)
                MinDist1 = norm(abs(edge(ii,1:2) - edge(jj,1:2)));
                if MinDist1<MinDist
                    MinDist = MinDist1;
                end
            end
        end
    end
end



    





% persistent   Free_x  Occ_x Free_y Occ_y 
% if ~exist('Rez','var')
%     Rez = 0.1;
% end
% edgeX = [];
% if ncFlag
%     Free_x = [Free_x SamplePoint(1)]; Free_y = [Free_y SamplePoint(2)];
% else
%     Occ_x = [Occ_x SamplePoint(1)]; Occ_y = [Occ_y SamplePoint(2)];
% end
% 
% if ~isempty(Free_x) && ~isempty(Occ_x)
%     kk = 1;
%     for jj = 1:length(Free_x)
%         for zz = 1:length(Occ_x)
%             Xdiff(kk) =  Free_x(jj) - Occ_x(zz);
%             Ydiff(kk) =  Free_y(jj) - Occ_y(zz);
%             kk = kk + 1;
%         end
%     end
%     
%     
%     for ii=1:length(Xdiff)
%         if abs(Xdiff(ii)) <= Rez && Ydiff(ii) == 0
%             if Occ_x(ii) > Free_x(ii)
%                 edgeX = Occ_x(ii) - Free_x(ii);
%             else
%                 edgeX = Free_x(ii) - Occ_x(ii);
%             end
%             Free_x(ii) = [];
%             Occ_x(ii)  = [];
%         end
%     end
% end
end