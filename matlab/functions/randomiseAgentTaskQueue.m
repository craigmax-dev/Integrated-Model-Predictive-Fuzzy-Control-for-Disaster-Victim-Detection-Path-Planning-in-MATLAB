function ini_params = randomiseAgentTaskQueue(n_a, n_q, n_x_s, n_y_s)
    % Initialize an empty array for ini_params
    ini_params = zeros(1, n_a * n_q * 2);

    % Fill ini_params with random integers within the specified ranges
    for i = 1:n_a * n_q
        ini_params(2*i-1) = randi([1, n_x_s], 1, 1); % x coordinate within [1, n_x_s]
        ini_params(2*i) = randi([1, n_y_s], 1, 1);   % y coordinate within [1, n_y_s]
    end
end
