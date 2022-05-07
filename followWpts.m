%%Follow RRTpath
%clean up
clearvars; close all; clc;
addpath("functions/")

scale =20;
current = [.2 0 0]';

% get waypoints
iterations = 2e3;
wptList = RRTstar(iterations,scale);

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
plotPtList(wptList,1,'#D95319');

xf = [0 0 0 0 0 .5 0 0 .1];
cur_pt = wptList(1,:);
next_pt = wptList(2,:);
% set the xy position
xf(1:2) = cur_pt;
%set psi
xf(5)=atan2(next_pt(2)-cur_pt(2),next_pt(1)-cur_pt(1));

traj.x = [];
traj.t = [];


% for each set of waypoints
for i=(1:length(wptList)-1)
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

    traj.x = [traj.x; segment.x];
    traj.t = [traj.t; segment.t];
    
    %get the last position as the new x0
    xf = segment.x(end,:);



end

%%
% plot the trajectory
figure(fig)

plotPtList(traj.x(:,1:2),1.5,'blue')


% plot the error



%%%%% FUNCTIONS

function out = plotPtList(ptList,width,color)

for i=1:length(ptList)-1
  pt = ptList(i,:);
  ppt = ptList(i+1,:);
  plot([ppt(1),pt(1)],[ppt(2),pt(2)],LineWidth=width,Color=color);
  hold on;
end

end



