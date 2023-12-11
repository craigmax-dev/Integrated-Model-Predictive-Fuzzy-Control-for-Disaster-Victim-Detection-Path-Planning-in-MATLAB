% V2.2 performance improvements refactor
function F = updateFireSpreadProbability(F, W, p, i, j, c_fs_1, c_fs_2, m_s, m_bo, n_x_e, n_y_e, r_w)
    % Determine the range for local update
    row_range = max(1, i - r_w) : min(n_x_e, i + r_w);
    col_range = max(1, j - r_w) : min(n_y_e, j + r_w);

    % Adjust indices in W to align with F's indices
    W_row_range = (row_range - i + r_w + 1);
    W_col_range = (col_range - j + r_w + 1);

    % Update the fire spread probability
    local_W = W(W_row_range, W_col_range);
    F(row_range, col_range) = F(row_range, col_range) + ...
                              c_fs_1 * (m_s(row_range, col_range) .* m_bo(row_range, col_range)) .* ...
                              local_W.^c_fs_2 * p;
end


% V2.1
% function F = updateFireSpreadProbability(F, W, p, i, j, c_fs_1, c_fs_2, m_s, m_bo, n_x_e, n_y_e, r_w)
% 
%     % Update the fire spread probability map F
%     for ii = 1:size(W, 1)
%         for jj = 1:size(W, 1)
%             iii = i + ii - r_w - 1;
%             jjj = j + jj - r_w - 1;
%             if iii > 0 && iii <= n_x_e && jjj > 0 && jjj <= n_y_e
%                 F(iii, jjj) = F(iii, jjj) + c_fs_1 * (m_s(iii, jjj) * m_bo(iii, jjj)) * W(ii, jj).^c_fs_2 * p;
%             end
%         end
%     end
% end