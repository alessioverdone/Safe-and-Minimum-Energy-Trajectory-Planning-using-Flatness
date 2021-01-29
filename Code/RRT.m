function final_points = RRT(obstacle)

seedsPerAxis = 3; %Number of seeds allowed on each axis (discretely placed seeds which idealy helps the RRT expansion)
%seedsPerAxis = 7;

treesMax = 2; %How many multiple trees (must be at least 2, 1 for source and 1 for destination
%treesMax = seedsPerAxis^3*3+2;

wallCount = 'obstacles.txt'; %Number of mock walls to be placed in the environment:
                             %no_obstacles.txt --> no obstacles in the environment
                             %obstacle.txt --> Three walls
                             %obstacles1.txt --> One wall
                             %obstacles2.txt --> Two walls
wallCount = obstacle;


rrt = RrtPlanner(treesMax,seedsPerAxis,wallCount)
rrt.SetStart([0 0 0]);
rrt.SetGoal([12 25 30]);
rrt.Run
hold on
plot3(rrt.smoothedPath(:,1),rrt.smoothedPath(:,2),rrt.smoothedPath(:,3),'k*');
%delete(rrt);
final_points=rrt.smoothedPath;
% disp(size(rrt.smoothedPath))

%To display the points of path and of the smoothed path, respectively command rrt.path and rrt.smoothedPath

