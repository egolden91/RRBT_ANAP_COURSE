function [ex,ey] = drawCovarianceEllipse(x,P,color, line_style, num_of_std,plotFlag)
% covarianceEllipse: plot a Gaussian as an uncertainty ellipse
% Based on Maybeck Vol 1, page 366
%
% num_of_std is an *optional* parameter that specifies how many standard
% deviations to draw: either 1 or 3. This parameter overrides k.
%
% Slightly modifed from covarianceEllipse by Vadim Indelman, July 2014. 

% k=2.296 corresponds to 1 std, 68.26% of all probability
% k=11.82 corresponds to 3 std, 99.74% of all probability
if nargin<6
    plotFlag = true;
end
k = 2.296;
if nargin >= 5
    if num_of_std == 1
        k = 2.296;
    elseif num_of_std == 3
        k = 11.82;
    else 
        error('Not currently supported');
    end
end

[e,s] = eig(P(1:2,1:2));
s1 = s(1,1);
s2 = s(2,2);
[ex,ey] = ellipse( sqrt(s1*k)*e(:,1), sqrt(s2*k)*e(:,2), x(1:2) );
if plotFlag
    h = line(ex,ey,'color',color,'linestyle',line_style);
end
function [x,y] = ellipse(a,b,c)
% ellipse: return the x and y coordinates for an ellipse
% [x,y] = ellipse(a,b,c);
% a, and b are the axes. c is the center

global ellipse_x ellipse_y
if ~exist('elipse_x')
    q =0:2*pi/25:2*pi;
    ellipse_x = cos(q);
    ellipse_y = sin(q);
end

points = a*ellipse_x + b*ellipse_y;
x = c(1) + points(1,:);
y = c(2) + points(2,:);

return