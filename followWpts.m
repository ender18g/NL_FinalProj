%%Follow RRTpath
%clean up
close all; clc;
addpath("functions/")

scale =20;
current = [.2 0 0]';

% get waypoints
iterations = 1e4;
%wptList = RRTstar(iterations,scale);

%% plot an environment

% define environment
boxSize = scale* 20;
safetyDist = 1;

% make obstacles
fig = figure;
obst{1} = scale*[-1 -boxSize/2 2 boxSize/2];
obst{2} = scale*[-4 1 8 5];

% plot obstacles
plotEnv(obst, boxSize);

%reverse order of points!
wptList=flip(wptList);

% plot the path
plot(wptList(:,1),wptList(:,2),LineWidth=1.5,LineStyle='--', ...
    Color='#EDB120',DisplayName='RRT* Path');

xf = [0 0 0 0 0 .5 0 0 .1];
cur_pt = wptList(1,:);
next_pt = wptList(2,:);
% set the xy position
xf(1:2) = cur_pt;
%set psi
xf(5)=atan2(next_pt(2)-cur_pt(2),next_pt(1)-cur_pt(1));

traj.x = [];
traj.t = [];
traj.X = [];

%create an mp4 video of plot
v = VideoWriter('traj_follow','MPEG-4');
v.FrameRate = 4;
open(v);



%% for each set of waypoints
for i=(1:length(wptList)-1)
%for i=1:3
    %the last xf is the new x0
    x0= xf;

    %the next xf
    xf = [0 0 0 0 0 .5 0 0 .1];
    cur_pt = wptList(i,:);
    next_pt = wptList(i+1,:);
    
    % set the xy position
    xf(1:2) = next_pt;
    
    %set theta
    xf(5)=atan2(next_pt(2)-cur_pt(2),next_pt(1)-cur_pt(1));

    %get the dist
    dist = norm(cur_pt-next_pt);

    % now simulate
    segment =TrajFollow(x0',xf',dist,current);

    traj.x = [traj.x; segment.x_act];
    traj.t = [traj.t; segment.t_act];
    traj.X = [traj.X segment.X_des];
    
    %get the last position as the new x0
    xf = segment.x_act(end,:);

    writeVideo(v,segment.frame);

end

close(v);

%%
% plot the desired trajectory
figure(fig)
plot(traj.X(1,:), traj.X(2,:),LineWidth=1.5,Color='red',LineStyle='--', ...
    DisplayName="Actual AUV Trajectory")

%%
% plot the trajectory
figure(fig)
plot(traj.x(:,1),traj.x(:,2),LineWidth=1,Color='blue', ...
    DisplayName="Generated Poly Fit Trajectory")
title("AUV Mission Execution")
subtitle("With [0.5 0 0] m/s Current")
xlabel('X (meters)')
ylabel('Y (meters)')
legend


% plot current
spacing = 30;

[mesh_X,mesh_Y] = meshgrid(-boxSize:spacing:boxSize,-boxSize:spacing:boxSize);
%quiver(mesh_X,mesh_Y,.5*ones(length(mesh_X)),zeros(length(mesh_X)));



