function global_map = set_local_map(local_map, agent_loc, r_local_maps, global_size)
    % SET_LOCAL_MAP Integrates a local map into a global map centered around the agent's location.
    %
    % Inputs:
    %   local_map    - The local map matrix to be integrated into the global map.
    %   agent_loc    - A two-element vector [x, y] representing the agent's location in the global map.
    %   r_local_maps - The radius of the local map. The local map is assumed to be of size (2*r_local_maps + 1) x (2*r_local_maps + 1).
    %   global_size  - A two-element vector [rows, cols] representing the size of the global map.
    %
    % Output:
    %   global_map   - The global map matrix with the local map integrated at the agent's location.
    %
    % Example usage:
    %   local_map = rand(7, 7);            % Example local map
    %   agent_loc = [50, 50];              % Example agent location
    %   r_local_maps = 3;                  % Radius of the local map
    %   global_size = [100, 100];          % Size of the global map
    %   global_map = set_local_map(local_map, agent_loc, r_local_maps, global_size);

    % Extract the x and y coordinates of the agent's location
    x_center = agent_loc(1);
    y_center = agent_loc(2);
    
    % Calculate the boundaries of the local map within the global map bounds
    x_start = max(1, x_center - r_local_maps);  % Start index for x-axis
    x_end = min(global_size(1), x_center + r_local_maps);  % End index for x-axis
    y_start = max(1, y_center - r_local_maps);  % Start index for y-axis
    y_end = min(global_size(2), y_center + r_local_maps);  % End index for y-axis
    
    % Initialize the global map with NaN values
    global_map = NaN(global_size);
    
    % Integrate the local map into the global map using the calculated boundaries
    global_map(x_start:x_end, y_start:y_end) = local_map;
end
