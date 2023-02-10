%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%This is Improved RRBT (scaled case for only 1 possible area to pass
%between obsticales) 
%The improved algorithm will save you time by calculating the minimum cov
%diameter and determening earlly on if safe passage is possible.
%
%look at lines 132 to 144 for detailed explanation
%In future versions we can upscale the improvment to cut out possible passage areas
%which are small to safely pass and by this not preform sampling in that
%area to save more time 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clearvars
close all
clear all
%%% MODEL %%%
F     = [1.0 0.0; 0.0 1.0];                 %Motion Model
sigMoveNoise   =  1^2*[1.0 0.0; 0.0 1.0];   %Model NOISE
sigObsrvNoise  =  0.1^2*[1.0 0.0; 0.0 1.0]; %OBSRV NOISE
muB0   =  [0,0];                            %INITIAL Estimation of belief
sigB0  =  [1,0;0,1];                        %INITIAL SIGMA of belief
lambdaB0 = sigB0;

[CovXTest,CovYTest]  = drawCovarianceEllipse([0 0],sigObsrvNoise+sigMoveNoise,'red','--',1,false);
minCovSize = max(CovYTest) - min(CovYTest);
%%%%%%%%%%%
%%
%%% MAP % OBSTICALS PARAMS %%% 
x_max = 50;
y_max = 50;
obstacle  = [0,25,23,10 ;...
            30,25,20,10 ];   % each row is an obs, LL corner pos coord , hight, width
obsrvArea = [20,0,30,20];    % pos where the robot can get an observation
EPS = 5;                     % max STEER dist 
numNodes = 2000;             % number of nodes that are randomley sampeled in the graph
refinRadius0 = 12;
delta = 1;
stohFlag = 1;
drawOnline = 0;
%%%%%%%%%%%%%

% INIT nodes
q_start.coord = [0 0];             
q_start.cost = 0;
q_start.parent = 0;
q_start.sigma = sigB0;
q_start.lambda = lambdaB0;
q_start.CovX = q_start.coord(1);
q_start.CovY = q_start.coord(2);
q_goal.coord = [49 45];
q_goal.cost = 0;
q_goal.sigma = sigB0;
q_goal.lambda = [0,0;0,0];
q_goal.CovX = 0;
q_goal.CovY = 0;


nodes(1) = q_start;
figure(1)
axis([0 x_max 0 y_max])
rectangle('Position',obsrvArea,'FaceColor',[1 1 1])
[obsLen,~] = size(obstacle);
for ii=1:obsLen
    ObsX(ii,:) =  [obstacle(ii,1) obstacle(ii,1) obstacle(ii,1)+obstacle(ii,3) obstacle(ii,1)+obstacle(ii,3)];
    ObsY(ii,:) =  [obstacle(ii,2) obstacle(ii,2)+obstacle(ii,4) obstacle(ii,2)+obstacle(ii,4) obstacle(ii,2)];
    rectangle('Position',obstacle(ii,:),'FaceColor',[0 .5 .5])
    hold on
    plot(q_goal.coord(1),q_goal.coord(2),'xr','LineWidth',2,'MarkerSize',10)
end
%%
StopFlag = 0;           % for loop inside for loop - the break statment only treats the inner loop
% del1 = 0;
% del2 = 10;
% [minDist,Px,Py] = FindNarrowAreas(ObsX,ObsY);

for i = 1:1:numNodes
    disp(i);
    %%% SAMPLE %%%
    %Improved sample%
%     if mod(i,5) == 0
%         q_rand = [floor(randi([min(Px)-del1 max(Px)+del1])) floor(randi([min(Py)-del2 max(Py)+del2]))];
%     else
    q_rand = [floor(rand(1)*x_max) floor(rand(1)*y_max)]; 
%     end
    plot(q_rand(1), q_rand(2), 'x', 'Color',  [0 0.4470 0.7410]) 
    %%%%%%%%%%%%%%
    
    %%% BREAK OPTION %%%
    % Break if goal node is already reached
    %Chance constraint 1 (12)
    for j = 1:1:length(nodes)
        if norm(nodes(j).coord - q_goal.coord) < EPS*0.1 %changed to bo proportional to EPS
            StopFlag = 1;
            break % this break is for the node check loop
        end
    end
    if StopFlag
        disp('Goal Reached')
        break % this break is for the whole algorithm loop
    end
    %%%%%%%%%%%%%%%%%%%%
    
    %%% NEAREST %%%
    % Pick the closest node from existing list to branch out from
    ndist = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        tmp = dist(n.coord, q_rand);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    q_near = nodes(idx);    %idx is the index of nearest insdie nodes list
    %%%%%%%%%%%%%%%%%%%%
    
    %%%% CONNECT %%%%
    q_new.coord = steer(q_rand, q_near.coord, val, EPS); % X nominal q_new.coord = muEstim 

    [ObsrvFlag] = CheckForObservation(obsrvArea,q_new.coord);
        if ObsrvFlag
            refinRadius = refinRadius0;
            [z] = GenerateObservation(q_new.coord,sigObsrvNoise);
            [~,~,q_new.sigma,K] = PropagateUpdateEstimation(q_new.coord,q_near.sigma,F,sigObsrvNoise,sigMoveNoise,z);
            [q_new.lambda]      = PropagateUpdateError(q_near.lambda,q_new.sigma,K,F);
        else
            refinRadius = refinRadius0;
            [~,~,q_new.sigma]   = PropagateEstimate(q_new.coord,q_near.sigma,F,sigMoveNoise);
            [q_new.lambda]      = PropagateError(q_near.lambda,F);
        end
        figure(1)
        [q_new.CovX,q_new.CovY]  = drawCovarianceEllipse(q_new.coord,(q_new.sigma+q_new.lambda),'red','--',1,false);
        hold on
        
    %%% Assuming our robot have a sensor that can detect corners. we sample
    %%% the envirment, when we detect a corner we save it and its
    %%% direction.
    %%% when we get two corners which are pointing at each other we
    %%% calculate the distance 
    %%% if we are not able to path through this distance safely taking into
    %%% account the minimum cov possible we call out an error that, that passage
    %%% is not possible
    %%% with more work we can expand this to prun out invalid passage areas
    %%% by not sampling in that area any more thus speeding up the
    %%% calculations 
    %%% also this code was writen for a scaled down case with only
    %%% rectangular obsticales and one passage area
    %%% but easilly expandable for larger scale scenarios.
    
    [MinDist] = FindObsEdge(q_new.coord,ObsX,ObsY,EPS);
    if minCovSize>MinDist && stohFlag == 1
        error("You'r uncertainty is to large you will never be able to pass the obsticale safely")
    end
    
        
    % CHANCE CONSTRAINT CHECKED HERE
    if ChanceConstraint([q_new.coord(1) q_new.coord(2)],q_near.coord,obstacle,ObsX,ObsY,q_new.CovX,q_new.CovY,delta,stohFlag) % cov of qnear checked and not the new rand node!!!!!!!
        line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], 'Color', 'k', 'LineWidth', 2); %connect to closest node
        [~,~]  = drawCovarianceEllipse(q_new.coord,(q_new.sigma+q_new.lambda),'green','--',1,drawOnline);
        drawnow
        hold on
        NominalX = q_new.coord;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %Chance constraint 2 (13)
        %check if cov ellipse hits an obsticale SIGMA = q_new.lambda+q_new.sigma

        q_new.cost = dist(q_new.coord, q_near.coord)*1 + det(q_new.lambda)+det(q_new.sigma)*100 + q_near.cost*100; % the cost of prev is not neccecery
        TestArray{i} = q_new;
        % Within a radius of r, find all existing nodes
        q_nearest = [];
        r = refinRadius;
        neighbor_count = 1;
        for j = 1:1:length(nodes)
            %if noCollision(nodes(j).coord, q_new.coord, obstacle) && dist(nodes(j).coord, q_new.coord) <= r
            if ChanceConstraint([q_new.coord(1) q_new.coord(2)],q_near.coord,obstacle,ObsX,ObsY,q_new.CovX,q_new.CovY,delta,stohFlag) && dist(nodes(j).coord, q_new.coord) <= r
                q_nearest(neighbor_count).coord = nodes(j).coord;
                q_nearest(neighbor_count).cost = nodes(j).cost;
                q_nearest(neighbor_count).CovX = nodes(j).CovX;
                q_nearest(neighbor_count).CovY = nodes(j).CovY;
                neighbor_count = neighbor_count+1;
            end
        end
        
        % Initialize cost to currently known value
        q_min = q_near;
        C_min = q_new.cost;
        
        % Iterate through all nearest neighbors to find alternate lower
        % cost paths
        
        for k = 1:1:length(q_nearest)
            %if noCollision(q_nearest(k).coord, q_new.coord, obstacle) && q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord) < C_min
            if ChanceConstraint([q_nearest(k).coord(1) q_nearest(k).coord(2)],q_new.coord,obstacle,ObsX,ObsY,q_new.CovX,q_new.CovY,delta,stohFlag) && q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord) < C_min
                q_min = q_nearest(k);
                C_min = q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord);
                line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], 'Color', 'g');                
                hold on
            end
        end
        
        % Update parent to least cost-from node
        for j = 1:1:length(nodes)
            if nodes(j).coord == q_min.coord
                q_new.parent = j;
            end
        end
        
        % Append to nodes
        nodes = [nodes q_new];
    end
end

D = [];
for j = 1:1:length(nodes)
    tmpdist = dist(nodes(j).coord, q_goal.coord);
    D = [D tmpdist];
end

% Search backwards from goal to start to find the optimal least cost path
[val, idx] = min(D);
if val<EPS
q_final = nodes(idx);
q_goal.parent = idx;
q_end = q_goal;
nodes_checkLength = [nodes q_goal];
while q_end.parent ~= 0
    start = q_end.parent;
    line([q_end.coord(1), nodes_checkLength(start).coord(1)], [q_end.coord(2), nodes_checkLength(start).coord(2)], 'Color', 'r', 'LineWidth', 2);
    drawCovarianceEllipse(nodes_checkLength(start).coord,nodes_checkLength(start).sigma+nodes_checkLength(start).lambda,'red','--',1,true);
    hold on
    q_end = nodes_checkLength(start);
end
end

