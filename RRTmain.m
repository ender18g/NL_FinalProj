%% Develop an RRT algorithm for AUV
clearvars; close all; clc;

% set number of iterations
iterations = 1e3;

%% plot an environment

% define environment
boxSize = 20;
safetyDist = .5;

% make obstacles
fig = figure;
obst{1} =[-2 -boxSize/2 4 boxSize/2];
obst{2} = [-1 1 2 4];

% plot obstacles
for i=1:numel(obst)
  rectangle('Position',obst{i},'Curvature',.2)
  hold on
  axis equal
end
% Set plot size
limits = [-boxSize/2 boxSize/2];
xlim(limits);
ylim(limits);

% Define waypoints
ip = [7,-9];
tgt =[-7,-9];

%radius for looking around to rewire
rad =.5;

% create database x, y, parent
nl(1).coord = ip;
nl(1).parent =0;
nl(1).cost = 0;
nl(1).dist=0;
nl(1).terminal=0;
nl(1).id = 1;

i=2;
tic;

%create an mp4 video of plot
% v = VideoWriter('RRT');
% v.FrameRate = 1;
% open(v);

%% Now for the iterations of RRTing
while(i<iterations)

  % gen point
  pt = randPt(boxSize,obst,safetyDist);

  % find closest parent and distance
  [parent,dist] = getParent(pt,nl);
  %[parent,dist] = getBestParent(pt,nl,rad);

  ppt = nl(parent).coord;

  %calculate total distance
  totalDist = nl(parent).cost + dist;

  % check collision along line
  if intersects(pt,ppt,obst,safetyDist)
    continue
  end

  %if parent is near tgt, add flag
  terminal = nearTgt(pt,tgt,rad);
  % add point to db
  nl(i).coord = pt;
  nl(i).parent =parent;
  nl(i).cost = totalDist;
  nl(i).terminal=terminal;
  nl(i).dist=dist;


  % plot new path

  plot([ppt(1),pt(1)],[ppt(2),pt(2)]);
  hold on;

  i=i+1;
  %calc percent complete and update plot
  if mod(i,500)==0
    complete = i/iterations;
    disp(complete*100);
    drawnow;

    %save to video
    %frame = getframe(fig);
    %writeVideo(v,frame);
  end



end

%% Now for publishing and cleanup
%find shortest path and plot
ptList = getShortest(nl,tgt);
plotPtList(ptList);

% Save the final trajectory for 5 frames
% for i=1:5
%   %save to video
%   frame = getframe(fig);
%   writeVideo(v,frame);
% end

%close(v);
toc





%% PROGRAM COMPLETE FUNCTIONS BELOW

function nl = rewire(nl,neighbors)

    % try to rewire
    if (numel(neighbors)>0)
      % see if any neighbors would benefit from new node as parent
      for i=1:length(neighbors)
        newCost = nl(i);
        if nl(i).cost
        end

      end

    end

    % make connections and update costs

end

function [bestParent,bestDist,neighbors] = getBestParent(pt,nl,rad)

    % define variables
    bestDist = norm(pt-nl(1).coord);
    bestCost = nl(1).cost + bestDist;
    bestParent = 1;
    betterParents = 0;
    numNeighbors = 0;

    %get all nodes around a radius
    for i=2:numel(nl)
      % get distance and cost
      dist = norm(pt-nl(i).coord);
      parentCost = nl(i).cost;
      %check if less than radius
      if (dist<rad)
        % we found a neighbor
        numNeighbors = numNeighbors + 1;
        neighbors(numNeighbors) = nl(i);

        if (parentCost<bestCost)
          % we found a better parent
          betterParents = betterParents +1;
          bestParent = i;
          bestDist = dist;
        end
      end
    end

    % if we didnt get any better parents in radius, go to old fx
    if (betterParents<1)
      [bestParent, bestDist] = getParent(pt,nl);
    end
  end

function out = plotPtList(ptList)

for i=1:length(ptList)-1
  pt = ptList(i,:);
  ppt = ptList(i+1,:);
  plot([ppt(1),pt(1)],[ppt(2),pt(2)],LineWidth=2,Color='blue');
end

end

function ptList = getShortest(nl,tgt)
minDist = 1e9;

%find the shortest total distance of terminal points
for i=1:numel(nl)
  if (nl(i).terminal ==1)
    if(nl(i).cost<minDist)
      bestFinal = i;
      minDist = nl(i).cost;
    end
  end
end

ptList(1,:) = tgt;
nextItem = bestFinal;

%make a list of all intermediate points
i=2;
while(1)
  ptList(i,:) = nl(nextItem).coord;
  nextItem = nl(nextItem).parent;

  %if you hit the first point, exit!
  if (nextItem == 0)
    return
  end

  i = i+1;
end


end

function terminal = nearTgt(pt,tgt,rad)
terminal =0;
dist = norm(pt-tgt);
if (dist<rad)
  terminal=1;
end
end

function [parent,minDist] = getParent(pt,nl)

minDist=norm(pt-nl(1).coord);
parent=1;

for i=2:numel(nl)
  %get distance
  dist = norm(pt-nl(i).coord);
  %check if min distance
  if (dist<minDist)
    minDist = dist;
    parent = i;
  end
end
end

function pt = randPt(boxSize,obst,safetyDist)
while(1)
  x = rand * boxSize - boxSize/2;
  y = rand * boxSize - boxSize/2;
  pt = [x,y];
  if (~collides(pt,obst,safetyDist))
    return
  end

end
end

function out = collides(pt,obst,safetyDist)

x = pt(1);
y = pt(2);

% assume that it collides
out = 1;
for i=1:numel(obst)

  % get coordinates
  corner_x = obst{i}(1);
  corner_y = obst{i}(2);
  width = obst{i}(3);
  height = obst{i}(4);

  %check if x inside bounds
  if ((x>=corner_x-safetyDist) && (x<=corner_x + width + safetyDist)...
      && (y>=corner_y-safetyDist) && (y<=corner_y + height + safetyDist))
    return
  end


end
% return false if no collision
out =0;

end

function out = intersects(parent,child,obst,safetyDist)
numPts = 100;

x_arr = linspace(parent(1),child(1),numPts);
y_arr = linspace(parent(2),child(2),numPts);

%assume collision
out = 1;
for i=1:length(x_arr)
  pt=[x_arr(i),y_arr(i)];
  if(collides(pt,obst,safetyDist))
    return
  end
end
out=0;
end