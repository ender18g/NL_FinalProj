function wptList = RRTstar(iterations,scale)


%% Develop an RRT algorithm for AUV



%% plot an environment

% define environment
boxSize = scale*20;
safetyDist = scale*.7;

% make obstacles
fig = figure;
obst{1} = scale*[-1 -boxSize/2 2 boxSize/2];
obst{2} = scale*[-4 1 8 5];

% plot obstacles
plotEnv(obst, boxSize);


% Define waypoints
ip = scale*[2,-9];
tgt =scale *[-2,-9];

%radius for looking around to rewire
rad =scale*1;

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
v = VideoWriter('RRTstar','MPEG-4');
v.FrameRate = 1;
open(v);

%% Now for the iterations of RRTing
while(i<iterations)

  % gen point
  pt = randPt(boxSize,obst,safetyDist);

  % find closest parent and distance
  [parent,dist,neighbors] = getBestParent(pt,nl,rad);

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
  nl(i).id =i;

  % now rewire
  if (class(neighbors)=='struct')
    nl = rewire(nl,neighbors);
  end

  i=i+1;
  %calc percent complete and update plot
  if mod(i,500)==0
    complete = i/iterations;
    disp(complete*100);
    try
    wptList = plotAll(nl,obst,boxSize,tgt);
    drawnow;
    %save to video
    frame = getframe(fig);
    writeVideo(v,frame);
    catch
        disp('no good route')
    end
  end



end



% Save the final trajectory for 5 frames
for i=1:3
  %save to video
  frame = getframe(fig);
  writeVideo(v,frame);
end

close(v);
toc



end

%% PROGRAM COMPLETE FUNCTIONS BELOW



function out = plotAll(nl,obst,boxSize,tgt)
clf;
plotEnv(obst,boxSize);
hold on;
for i=numel(nl):-1:1
  pt = nl(i).coord;
  parentId = nl(i).parent;

  % if parent is the first ip, stop!
  if parentId ==0
    break
  end
  ppt = nl(parentId).coord;
  % plot new pat
  plot([ppt(1),pt(1)],[ppt(2),pt(2)],Color='#4DBEEE');
  hold on;
end

%% Now for publishing and cleanup
%find shortest path and plot
ptList = getShortest(nl,tgt);
plotPtList(ptList);
out = ptList;
end

function nl = rewire(nl,neighbors)

% get the newest point
pt = nl(end);

% see if any neighbors would benefit from new node as parent
for i=1:numel(neighbors)
  if (~isfield(neighbors(i),'id'))
    break;
  end
  %get the id of the neighbor
  id = neighbors(i).id;
  oldCost = nl(id).cost;
  dist = norm(pt.coord - nl(id).coord);

  %calculate cost with new point
  newCost = pt.cost + dist;

  % if the new cost is better, REWIRE
  if (newCost < oldCost)
    % assign pt as the new parent
    nl(id).parent = pt.id;
    nl(id).cost = newCost;
    nl(id).dist = dist;

    %change any children of this node with new costs
    nl = changeChildren(id,nl);


  end

end



  function nl = changeChildren(parentId,nl)
    parent = nl(parentId);
    for j=1:numel(nl)
      % find children
      if (nl(j).parent == parent.id)
        % change the childrens' cost
        nl(j).cost = parent.cost + nl(j).dist;

        %change all grandchildren
        changeChildren(nl(j).id,nl);
      end
    end

  end

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
    % we found a neighbor add to neighbor array
    numNeighbors = numNeighbors + 1;
    neighbors(numNeighbors)=nl(i);

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

%if no neighbors found, return 0;
if(~exist("neighbors"))
  neighbors = 0;
end


end

function out = plotPtList(ptList)

for i=1:length(ptList)-1
  pt = ptList(i,:);
  ppt = ptList(i+1,:);
  plot([ppt(1),pt(1)],[ppt(2),pt(2)],LineWidth=4,Color='#D95319');
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