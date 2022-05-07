%%Follow RRTpath
%clean up
clearvars; close all; clc;
addpath("functions/")

scale =20;

% get waypoints
iterations = 5e2;
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

% plot the path
plotPtList(wptList);

xf = [0 0 0 0 0 .5 0 0 .1];
cur_pt = wptList(1,:);
next_pt = wptList(2,:);
% set the xy position
xf(1:2) = cur_pt;
%set psi
xf(5)=atan2(next_pt(2)-cur_pt(2),next_pt(1)-cur_pt(1));

% for each set of waypoints
for i=1:10
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
    TrajFollow(x0',xf',dist);

end


% plot the trajectory


% plot the error



%%%%% FUNCTIONS

function out = plotPtList(ptList)

for i=1:length(ptList)-1
  pt = ptList(i,:);
  ppt = ptList(i+1,:);
  plot([ppt(1),pt(1)],[ppt(2),pt(2)],LineWidth=2,Color='#D95319');
  hold on;
end

end



