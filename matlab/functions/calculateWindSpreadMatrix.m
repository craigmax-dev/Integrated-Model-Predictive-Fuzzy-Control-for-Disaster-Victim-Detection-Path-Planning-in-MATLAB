function W = calculateWindSpreadMatrix(r_w, c_wm_1, c_wm_2, c_wm_d, ang_w, v_w)

  w_dir = zeros(2*r_w + 1, 2*r_w + 1);
  w_dis = zeros(2*r_w + 1, 2*r_w + 1);

  for i = 1:(2*r_w + 1)
    for j = 1:(2*r_w + 1)
      f_d = atan2((i-(r_w+1)),(j-(r_w+1)));
      ang = ang_w - f_d;
      w_dir(i, j) = exp(v_w * (c_wm_1 + c_wm_2 * (cos(ang) - 1)));
      w_dis(i, j) = c_wm_d ^ sqrt((i - (r_w + 1))^2 + (j - (r_w + 1))^2);
    end
  end

  W = w_dir .* w_dis;
end
