% Team: Cap
function path = Task_1(mapped, startPoint, stopPoint)
    path = astar_algorithm(mapped, startPoint, stopPoint);
end

function path = astar_algorithm(map, startPoint, stopPoint)
  
    % Define the movement costs
    diagonalCost = sqrt(2);
    straightCost = 1;
    
    % Initialize open and closed lists
    openList = [];
    closedList = false(size(map));
    
    % Extract start and stop coordinates
    startCoord = startPoint;
    stopCoord = stopPoint;
    
    % Create a node structure for the start node
    startNode = struct('coord', startCoord, 'parent', [], ...
                      'g', 0, 'h', heuristic(startCoord, stopCoord), 'f', 0);
    
    % Add the start node to the open list
    openList = [openList, startNode];
    
    % Loop until the open list is empty
    while ~isempty(openList)
        % Find the node with the lowest f cost in the open list
        [~, idx] = min([openList.f]);
        currentNode = openList(idx);
        
        % Remove the current node from the open list
        openList(idx) = [];
        
        % Mark the current node as closed
        closedList(currentNode.coord(1), currentNode.coord(2)) = true;
        
        % Check if the current node is the goal
        if isequal(currentNode.coord, stopCoord)
            path = reconstructPath(currentNode);
            return;
        end
        
        % Generate neighboring nodes
        neighbors = generateNeighbors(currentNode.coord, map);
        
        % Iterate through neighbors
        for i = 1:size(neighbors, 1)
            neighborCoord = neighbors(i, :);
            
            % Skip if the neighbor is not a valid position
            if ~isValid(neighborCoord, map) || closedList(neighborCoord(1), neighborCoord(2))
                continue;
            end
            
            % Calculate tentative g score
            tentativeG = currentNode.g + calculateCost(currentNode.coord, neighborCoord, map);
            
            % Check if the neighbor is not in the open list or has a lower g score
            neighborIndex = findCoordInList(neighborCoord, openList);
            if isempty(neighborIndex) || tentativeG < openList(neighborIndex).g
                % Create the neighbor node
                neighborNode = struct('coord', neighborCoord, 'parent', currentNode, ...
                                      'g', tentativeG, 'h', heuristic(neighborCoord, stopCoord), 'f', 0);
                
                % Update f score
                neighborNode.f = neighborNode.g + neighborNode.h;
                
                % Add the neighbor to the open list
                openList = [openList, neighborNode];
            end
        end
    end
    
    % If the open list is empty and goal is not reached, return an empty path
    path = [];
end

function h = heuristic(coord, goalCoord)
    % Euclidean distance heuristic
    h = norm(coord - goalCoord);
end

function cost = calculateCost(coord1, coord2, map)
    % Calculate the cost between two coordinates
    speedLimitCost = 1 / map.speedLimitCost(coord1(1), coord1(2));
    trafficIntensity = map.trafficIntensity(coord1(1), coord1(2));
    obstacleCost = map.obstacleCost(coord1(1), coord1(2));
    
    % Calculate cost based on the formula
    cost = speedLimitCost * trafficIntensity + obstacleCost;
    
    % Adjust cost for diagonal movement
    if norm(coord2 - coord1) > 1
        cost = cost * sqrt(2);
    end
end

function neighbors = generateNeighbors(coord, map)
    % Generate neighboring coordinates
    [rows, cols] = size(map.road);
    [dx, dy] = meshgrid(-1:1, -1:1);
    neighbors = [coord(1) + dx(:), coord(2) + dy(:)];
    
    % Filter out invalid neighbors
    validIndices = isValid(neighbors, map);
    neighbors = neighbors(validIndices, :);
    
    % Exclude the current position
    neighbors = neighbors(~all(neighbors == coord, 2), :);
end

function validIndices = isValid(coords, map)
    % Check if coordinates are within the map bounds and on a road
    validIndices = all(coords > 0, 2) & all(coords <= [size(map.roadMaproad, 1), size(map.road, 2)], 2) ...
        & map.road(sub2ind(size(map.road), coords(:, 1), coords(:, 2)));
end

function path = reconstructPath(node)
    % Reconstruct the path from the goal node to the start node
    path = [];
    while ~isempty(node)
        path = [node.coord; path];
        node = node.parent;
    end
end

function index = findCoordInList(coord, nodeList)
    % Find the index of a coordinate in a list of nodes
    indices = arrayfun(@(x) isequal(x.coord, coord), nodeList);
    index = find(indices, 1);
end
