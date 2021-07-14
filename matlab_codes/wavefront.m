function trajectory = wavefront(pos, goal, xaxis, yaxis, logitMap)

% Create a cost map
cost = zeros(size(logitMap));

% Coordinate of goal and current pos:
[~, goalx] = min(abs(xaxis - goal(1)));
[~, goaly] = min(abs(yaxis - goal(2)));
[~, posx] = min(abs(xaxis - pos(1)));
[~, posy] = min(abs(yaxis - pos(2)));

open = [goalx goaly];
cost(goalx, goaly) = 1;
adjacent_grids = [ 1 0; 0 1; -1 0; 0 -1];

while size(open,1) ~= 0
 
  for k=1:size(adjacent_grids,1)
    
    adjacent = open(1,:)+adjacent_grids(k,:); % Calculate index for current adjacent cell:
    
    if min(adjacent) < 1 % Make sure cell is in the map
      continue
    end
    
    if adjacent(1) > length(xaxis) % Make sure cell is in the map
      continue
    end
    
    if adjacent(2) > length(yaxis) % Make sure cell is in the map
      continue
    end
   
    if logitMap(adjacent(1), adjacent(2)) == 1  % Make sure the adjacent cell is not an obstacle 
      continue
    end
    
    if cost(adjacent(1), adjacent(2)) ~= 0 % Make sure the adjacent cell is not closed
      continue
    end
    
    % Set the cost and add the adjacent to the open set
    cost(adjacent(1), adjacent(2)) = cost(open(1,1), open(1,2)) + 1;
    open(size(open,1)+1,:) = adjacent;
    
  end
  % Pop the top open cell from the queue
  open = open(2:end,:);
end

% Trajectory calculation
trajectory = [posx; posy];
while 1
  i = size(trajectory,2);
  
  if trajectory(:,i) == [goalx; goaly]
    break
  end

   curc = cost(trajectory(1,i),trajectory(2,i));

  noMove = 1;
  for k=1:size(adjacent_grids,1)
    % Calculate index for current adjacent cell:
    adjacent = trajectory(:,i)+adjacent_grids(k,:)';
   
    if min(adjacent) < 1  % Make sure adjacent cell is in the map
      continue
    end
    
    if adjacent(1) > length(xaxis)  % Make sure adjacent cell is in the map
      continue
    end
    
    if adjacent(2) > length(yaxis)  % Make sure adjacent cell is in the map
      continue
    end
   
   
    if cost(adjacent(1),adjacent(2)) == 0  % If this adjacent cell reduces cost, add it to the path.
      continue;
    end
    
    if cost(adjacent(1),adjacent(2)) < curc
      noMove = 0;
      trajectory(:,i+1) = adjacent;
      break
    end
  end
  
  if noMove
    trajectory = [posx; posy];
    break;
  end
end
trajectory = transpose(trajectory);