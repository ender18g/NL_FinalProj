%%Follow RRTpath
%clean up
clearvars; close all; clc;
addpath("functions/")

scale =20;

% get waypoints
iterations = 2e3;
wptList = RRTstar(iterations,scale);

%% plot an environment

% define environment
boxSize = scale* 20;
safetyDist = 1;

% make obstacles
fig = figure;
obst{1} =scale*[-1 -boxSize/2 2 boxSize/2];
obst{2} = scale*[-4 1 8 5];

% plot obstacles
plotEnv(obst, boxSize);

% plot the path
plotPtList(wptList);


% for each set of waypoints


% set initial conditions


% get trajectory


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



