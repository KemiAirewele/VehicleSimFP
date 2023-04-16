function get_current_segment(localized_coords, map_segments)
    x = localized_coords.lat
    y = localized_coords.long
    segs = []
    for seg in map_segments
        p1 = seg[2].lane_boundaries[1].pt_a
        p2 = seg[2].lane_boundaries[1].pt_b
        p3 = seg[2].lane_boundaries[2].pt_a
        p4 = seg[2].lane_boundaries[2].pt_b
        max_x = max(p1[1], p2[1], p3[1], p4[1])
        min_x = min(p1[1], p2[1], p3[1], p4[1])
        max_y = max(p1[2], p2[2], p3[2], p4[2])
        min_y = min(p1[2], p2[2], p3[2], p4[2])
        if min_x-5 < x < max_x+5 && min_y-5 < y < max_y+5
            append!(segs, seg[1])
        end
    end
    return segs
end

function get_path(start_map_segment, target_map_segment, map_segments, seg_lengths, max_id)
    shortest_to = zeros(Float64, max_id)
    prevs = zeros(Int, max_id)
    for i in 1:max_id
        shortest_to[i] = 100000
    end

    pqueue = PriorityQueue{Any, Float64}()
    enqueue!(pqueue, [start_map_segment, -1], 0)
    shortest_to[start_map_segment] = 0
    prevs[start_map_segment] = -1
    visited = zeros(Int, max_id)
    seg_id = start_map_segment
    while length(pqueue) > 0 && seg_id != target_map_segment
        key = dequeue!(pqueue)
        seg_id = key[1]
        prev = key[2]
        visited[seg_id] = 1

        for child in map_segments[seg_id].children
            if visited[child] == 0
                if seg_lengths[child] + shortest_to[seg_id] < shortest_to[child]
                    shortest_to[child] = seg_lengths[child] + shortest_to[seg_id]
                    prevs[child] = seg_id
                end
                enqueue!(pqueue, [child, seg_id], shortest_to[child])
                visited[child] = 1
            end
        end
    end

    path = [target_map_segment]
    while path[length(path)] != start_map_segment
        append!(path, prevs[path[length(path)]])
    end

    return path
end

function build_mid_lengths(map_segments, max_id)
    mid_lengths = zeros(Float64, (max_id, 4))
    for seg in map_segments
        if abs(seg[2].lane_boundaries[1].curvature - 0) < 0.001
            # straight
            p1 = seg[2].lane_boundaries[1].pt_a
            p2 = seg[2].lane_boundaries[1].pt_b
            p3 = seg[2].lane_boundaries[2].pt_a
            p4 = seg[2].lane_boundaries[2].pt_b

            if abs(p2[1]-p1[1]) < 0.001
                # same x
                if p2[2] > p1[2]
                    # up
                    mid_lengths[seg[1],1] = 2
                else
                    # down
                    mid_lengths[seg[1],1] = 4
                end
                mid_lengths[seg[1],2] = (p1[1]+p3[1])/2
                if VehicleSim.loading_zone in seg[2].lane_types
                    mid_lengths[seg[1],3] = (p1[1]+p3[1])/2
                end
            else
                # same y
                if p2[1] > p1[1]
                    # right
                    mid_lengths[seg[1],1] = 1
                else
                    # left
                    mid_lengths[seg[1],1] = 3
                end
                mid_lengths[seg[1],2] = (p1[2]+p3[2])/2
            end
        else
            # curves
            p1 = seg[2].lane_boundaries[1].pt_a
            p2 = seg[2].lane_boundaries[1].pt_b
            p3 = seg[2].lane_boundaries[2].pt_a
            p4 = seg[2].lane_boundaries[2].pt_b

            if abs(p1[1]-p3[1]) < 0.001
                # same x
                x = p1[1]
                y = p2[2]
                rad = abs(p1[2]-y+p3[2]-y)/2
                if (p2[1] > x && p1[2] > y) || (p2[1] < x && p1[2] < y)
                    # clockwise
                    mid_lengths[seg[1],1] = 5
                else
                    # counter-clockwise
                    mid_lengths[seg[1],1] = 6
                end
                mid_lengths[seg[1],2] = rad
                mid_lengths[seg[1],3] = x
                mid_lengths[seg[1],4] = y
            else
                # same y
                x = p2[1]
                y = p1[2]
                rad = abs(p1[1]-x+p3[1]-x)/2
                if (p1[1] < x && p2[2] > y) || (p1[1] > x && p2[2] < y)
                    # clockwise
                    mid_lengths[seg[1],1] = 5
                else
                    # counter-clockwise
                    mid_lengths[seg[1],1] = 6
                end
                mid_lengths[seg[1],2] = rad
                mid_lengths[seg[1],3] = x
                mid_lengths[seg[1],4] = y
            end
        end
    end
    return mid_lengths
end

function build_seg_lengths(map_segments)
    seg_lengths = Dict{Int, Float64}()
    for seg in map_segments
        if seg[2].lane_types[1] == VehicleSim.standard || seg[2].lane_types[1] == VehicleSim.intersection
            if abs(seg[2].lane_boundaries[1].curvature - 0) < 0.001
                p1 = seg[2].lane_boundaries[1].pt_a
                p2 = seg[2].lane_boundaries[1].pt_b
                p3 = seg[2].lane_boundaries[2].pt_a
                p4 = seg[2].lane_boundaries[2].pt_b
                seg_lengths[seg[1]] = non_ten(p1, p2, p3, p4)
            else
                p1 = seg[2].lane_boundaries[1].pt_a
                p3 = seg[2].lane_boundaries[2].pt_a
                p2 = seg[2].lane_boundaries[1].pt_b
                if abs(p1[1]-p3[1]) < 0.001
                    # same x
                    x = p1[1]
                    y = p2[2]
                    rad = abs(p1[2]-y+p3[2]-y)/2
                    seg_lengths[seg[1]] = 2*pi*rad/4
                else
                    # same y
                    x = p2[1]
                    y = p1[2]
                    rad = abs(p1[1]-x+p3[1]-x)/2
                    seg_lengths[seg[1]] = 2*pi*rad/4
                end
            end
        elseif seg[2].lane_types[1] == VehicleSim.stop_sign
            p1 = seg[2].lane_boundaries[1].pt_a
            p2 = seg[2].lane_boundaries[1].pt_b
            p3 = seg[2].lane_boundaries[2].pt_a
            p4 = seg[2].lane_boundaries[2].pt_b
            seg_lengths[seg[1]] = non_ten(p1, p2, p3, p4)
        end
        if length(seg[2].lane_types) > 1 && seg[2].lane_types[2] == VehicleSim.loading_zone
            # TODO
        end
    end
    return seg_lengths
end

function non_ten(p1, p2, p3, p4)
    if abs(abs(norm(p1-p2))-10) > 0.01
        return abs(norm(p1-p2))
    elseif abs(abs(norm(p3-p2))-10) > 0.01
        return abs(norm(p3-p2))
    elseif abs(abs(norm(p4-p3))-10) > 0.01
        return abs(norm(p4-p3))
    end
    return abs(norm(p1-p4))
end

function build_adjacency_matrix(map_segments, max_id)
    mat = zeros(Int,(max_id, max_id))

    for seg in map_segments
        for child in seg[2].children
            mat[seg[2].id, child] = 1
        end
    end

    return mat
end

function get_yaw(orientation)
    q1 = orientation[1]
    q2 = orientation[2]
    q3 = orientation[3]
    q4 = orientation[4]
    t3 = 2.0 * (q1 * q4 + q2 * q3)
    t4 = 1.0 - 2.0 * (q3 * q3 + q4 * q4)
    yaw = atan(t3,t4)
    return yaw
end

function yaw_converter(yaw)
    return (yaw + 3*pi) % (2*pi) - pi
end

function get_steer!(target_yaw, current_yaw)
    perspective_yaw = yaw_converter(current_yaw - target_yaw)
    return -perspective_yaw
end
