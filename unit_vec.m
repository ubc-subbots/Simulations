function vec = unit_vec(dimension, nonzero_ind)
    vec = zeros(dimension,1);
    vec(nonzero_ind) = 1;
end