function potential = calculate_potential_map(attraction_gain, repulsion_gain, p0, CSpace_Goal, CSpace_Obstacles, x_axis, y_axis)
fprintf('Calculating potential field.');
tic

[x_grid, y_grid] = meshgrid(x_axis, y_axis);
CSpace_Grid = [x_grid(:) y_grid(:)];

[~,dist] = dsearchn(CSpace_Obstacles, CSpace_Grid);
dist(dist > p0) = p0;


repulsive_potential = 0.5 * repulsion_gain * ( (1 ./ dist) - (1 / p0) ) .^ 2;

attraction_distances = sum((CSpace_Grid - CSpace_Goal) .^ 2, 2);
attractive_potential = 0.5 * attraction_gain * attraction_distances;

potential =  repulsive_potential + attractive_potential;
potential(potential == Inf) = second_max(potential) + 1;
potential = reshape(potential,[length(y_axis) length(x_axis)]);

elapsed_time = toc;
fprintf('\nPotential field generation completed in %.2f seconds\n',elapsed_time);

    function  y  = second_max(x)
        y = max(x(x < max(x)));
    end
end