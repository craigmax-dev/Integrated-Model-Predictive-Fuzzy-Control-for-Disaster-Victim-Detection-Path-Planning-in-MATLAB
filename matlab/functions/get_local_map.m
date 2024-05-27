function local_map = get_local_map(global_map, agent_loc, r_local_maps)
    % GET_LOCAL_MAP Extracts a local map from the global map centered around the agent's location.
    %
    % Inputs:
    %   global_map    - The global map matrix from which the local map is to be extracted.
    %   agent_loc     - A two-element vector [x, y] representing the agent's location in the global map.
    %   r_local_maps  - The radius of the local map. The local map will be of size (2*r_local_maps + 1) x (2*r_local_maps + 1).
    %
    % Output:
    %   local_map     - The extracted local map matrix centered around the agent's location.
    %
    % Example usage:
    %   global_map = rand(100, 100);   % Example global map
    %   agent_loc = [50, 50];          % Example agent location
    %   r_local_maps = 3;              % Radius of the local map
    %   local_map = get_local_map(global_map, agent_loc, r_local_maps);
    
    % Extract the x and y coordinates of the agent's location
    x_center = agent_loc(1);
    y_center = agent_loc(2);
    
    % Calculate the boundaries of the local map, ensuring they are within the global map bounds
    x_start = max(1, x_center - r_local_maps);  % Start index for x-axis
    x_end = min(size(global_map, 1), x_center + r_local_maps);  % End index for x-axis
    y_start = max(1, y_center - r_local_maps);  % Start index for y-axis
    y_end = min(size(global_map, 2), y_center + r_local_maps);  % End index for y-axis
    
    % Extract the local map from the global map using the calculated boundaries
    local_map = global_map(x_start:x_end, y_start:y_end);
end
