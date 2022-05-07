function out = plotEnv(obst,boxSize)
% plot obstacles
for i=1:numel(obst)
  rectangle('Position',obst{i},'Curvature',.2,'EdgeColor',[0 0.4470 0.7410], 'FaceColor',[0.9290 0.6940 0.1250], "LineWidth",1)
  hold on
  axis equal
end

% Set plot size
limits = [-boxSize/2 boxSize/2];
title('RRT* Path Planning')
xlim(limits);
ylim(limits);

%plot ship
ship = imread('ship.jpg');
x = obst{2}(1);
y = obst{2}(2);
w = obst{2}(3);
h = obst{2}(4);

buf = .1;

image([x-buf x+w+buf],[y+h+buf y-buf],ship);

end