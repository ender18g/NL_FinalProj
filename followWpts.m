%%Follow RRTpath
%clean up
close all; clc;
addpath("functions/")

scale =20;
current = .25*[1 1 0]';

% get waypoints
iterations = 5e3;
wptList = RRTstar(iterations,scale);
wptList = flip(wptList);

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

% plot the path
plot(wptList(:,1),wptList(:,2),LineWidth=1.5,LineStyle='--', ...
    Color='#EDB120',DisplayName='RRT* Path');

%now add a gradual descent and climb to surface
wptList = [wptList(:,1:2), zeros(length(wptList),1)];
surface = 15;
climb_d = 200;
slope = surface/climb_d;

%set the beginning and end at surface
wptList(1,3) = surface;
wptList(end,3)=surface;


wptList=flip(wptList);
%this loop is for the climb
for i=1:(length(wptList)-1)
    cur_pt = wptList(i,:);
    next_pt = wptList(i+1,:);

    %get the horizontal dist
    dist = norm(cur_pt-next_pt);
    current_z = cur_pt(3);

    % if you arent a 0, go to 0 max dist
    if current_z>0
        next_d=current_z - dist*(slope);
        if next_d<0
            next_d = 0;
            %made 0 break!
            wptList(i+1,3)=next_d;
            break;
        end
        %set the desired depth for the next point
        wptList(i+1,3)=next_d;
    end

end

%reverse order of points!
wptList=flip(wptList);

%this loop is for the descent
for i=1:(length(wptList)-1)
    cur_pt = wptList(i,:);
    next_pt = wptList(i+1,:);

    %get the horizontal dist
    dist = norm(cur_pt-next_pt);
    current_z = cur_pt(3);

    % if you arent a 0, go to 0 max dist
    if current_z>0
        next_d=current_z - dist*(slope);
        if next_d<0
            next_d = 0;
            wptList(i+1,3)=next_d;
            break %made it to 0, break!
        end
        %set the desired depth for the next point
        wptList(i+1,3)=next_d;
    end

end


% setup initial condition
xf = [0 0 0 0 0 1 0 0 0];
cur_pt = wptList(1,:);
next_pt = wptList(2,:);
% set the xy position
xf(1:3) = cur_pt;
%set psi
xf(5)=atan2(next_pt(2)-cur_pt(2),next_pt(1)-cur_pt(1));

traj.x = [];
traj.t = [];
traj.X = [];

%create an mp4 video of plot
v = VideoWriter('traj_follow','MPEG-4');
v.FrameRate = 6;
open(v);

%% for each set of waypoints
for i=(1:length(wptList)-1)
    %for i=1:3
    %the last xf is the new x0
    x0= xf;

    cur_pt = wptList(i,:);
    next_pt = wptList(i+1,:);

    %get the dist
    dist = norm(cur_pt-next_pt);

    %the next xf
    xf = [0 0 0 0 0 1 0 0 0];


    % set the xyz position
    xf(1:3) = next_pt;

    %if climbing, raise the angle
    if xf(3)-x0(3)>1
        xf(4)=-tan(surface/climb_d);
    end

    if xf(3)-x0(3)<-1
        xf(4)=tan(surface/climb_d);
    end

    %set theta
    xf(5)=atan2(next_pt(2)-cur_pt(2),next_pt(1)-cur_pt(1));



    % now simulate
    segment =TrajFollow(x0',xf',.5*dist,current);

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
    DisplayName="Generated Poly Fit Trajectory")

%%
% plot the trajectory
figure(fig)
plot(traj.x(:,1),traj.x(:,2),LineWidth=1,Color='blue', ...
    DisplayName="Actual AUV Trajectory")
title("AUV Mission Execution")
subtitle("With [1 1 0] m/s Current")
xlabel('X (meters)')
ylabel('Y (meters)')
grid on;
legend


% plot current
spacing = 30;

[mesh_X,mesh_Y] = meshgrid(-boxSize:spacing:boxSize,-boxSize:spacing:boxSize);
%quiver(mesh_X,mesh_Y,.5*ones(length(mesh_X)),zeros(length(mesh_X)));



