% Base deiniions for the Path Planner

% Flag indicating whether the path has changed
NewPath = 0;

%Flag indicating whether the world has changed and a new path is required
WorldChanged = 1;

% List of goal locations 
goals = [7.0, 7.0];

% Path waypoint list
path = zeros(size(OccGrid,1)*size(OccGrid, 2), 2);
% Length of the path
pathlen = 0;


