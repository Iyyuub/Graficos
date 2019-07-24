function [ target_indices target_distances unassigned_targets ] = nearestneighborlinker(source, target, max_distance)
    if nargin < 3
        max_distance = Inf;
    end
    n_source_points = size(source, 1);
    n_target_points = size(target, 1);
    D = NaN(n_source_points, n_target_points);
    % Build distance matrix
    for i = 1 : n_source_points
        % Pick one source point
        current_point = source(i, :);
        % Compute square distance to all target points
        diff_coords = target - repmat(current_point, n_target_points, 1);
        square_dist = sum(diff_coords.^2, 2);
        % Store them
        D(i, :) = square_dist;
    end
     % Deal with maximal linking distance: we simply mark these links as already
    % treated, so that they can never generate a link.
    D ( D > max_distance * max_distance ) = Inf;
    target_indices = -1 * ones(n_source_points, 1);
    target_distances = NaN(n_source_points, 1);
    % Parse distance matrix
    while ~all(isinf(D(:)))
        [ min_D closest_targets ] = min(D, [], 2); % index of the closest target for each source points
        [ ~, sorted_index ] = sort(min_D);
        for i = 1 : numel(sorted_index)
            source_index =  sorted_index(i);
            target_index =  closest_targets ( sorted_index(i) );
            % Did we already assigned this target to a source?
            if any ( target_index == target_indices )
                % Yes, then exit the loop and change the distance matrix to
                % prevent this assignment
                break
            else
                % No, then store this assignment
                target_indices( source_index ) = target_index;
                target_distances ( source_index ) = sqrt ( min_D (  sorted_index(i) ) );
                % And make it impossible to find it again by putting the target
                % point to infinity in the distance matrix
                D(:, target_index) = Inf;
                % And the same for the source line
                D(source_index, :) = Inf;
            end
        end
    end
    unassigned_targets = setdiff ( 1 : n_target_points , target_indices );
        