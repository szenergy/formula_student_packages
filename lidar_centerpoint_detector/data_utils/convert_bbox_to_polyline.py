import argparse
import pathlib
import json
import numpy as np
import math

from pathlib import Path
from sklearn.cluster import SpectralClustering
from scipy.spatial.transform import Rotation


# def cluster_cones_into_borders(cones, max_distance=5.0, max_angle_change=np.pi/4):
#     """
#     Cluster cones into borders using spectral clustering.
#     """
#     if not cones or len(cones) < 2:
#         return [cones] if cones else []
    
#     # Convert to numpy array
#     cones_array = np.array(cones)
    
#     # Determine the number of borders
#     num_borders = determine_number_of_borders(cones, max_distance)
    
#     # Adjust n_neighbors to avoid errors (must be less than number of samples)
#     n_neighbors = min(len(cones_array) - 1, 5)  # Default is 10, which can cause errors with small datasets
    
#     # Apply Spectral Clustering
#     spectral = SpectralClustering(
#         n_clusters=num_borders,
#         affinity='nearest_neighbors', #'nearest_neighbors',
#         eigen_solver="arpack",
#         n_init=100,
#         degree=6,
#         n_neighbors=n_neighbors,
#         random_state=42
#     )
    
#     # cluster the points
#     labels = spectral.fit_predict(cones_array)
    
#     # Group cones by cluster label
#     borders = [[] for _ in range(num_borders)]
#     for i, label in enumerate(labels):
#         borders[label].append(cones_array[i])
    
#     # Sort each border by x-coordinate
#     for i in range(len(borders)):
#         borders[i] = sorted(borders[i], key=lambda p: p[0])
    
#     return borders

# def group_cones_into_polylines(data_dict):
#     grouped_polylines = {}
    
#     for label_id, cones in data_dict.items():
#         grouped_polylines[label_id] = {}
        
#         # Process blue cones (Left Border)
#         if 'Cone_Blue' in cones:
#             blue_cones = cones['Cone_Blue']
#             left_borders = cluster_cones_into_borders(blue_cones, max_borders=2)
#             for i, border in enumerate(left_borders):
#                 grouped_polylines[label_id][f'Left_Border_{i+1}'] = border
        
#         # Process yellow cones (Right Border)
#         if 'Cone_Yellow' in cones:
#             yellow_cones = cones['Cone_Yellow']
#             right_borders = cluster_cones_into_borders(yellow_cones, max_borders=2)
#             for i, border in enumerate(right_borders):
#                 grouped_polylines[label_id][f'Right_Border_{i+1}'] = border
        
#         # Process big cones (Start)
#         if 'Cone_Big' in cones:
#             grouped_polylines[label_id]['Start'] = cones['Cone_Big']
    
#     return grouped_polylines

# def cluster_cones_into_borders(cones, max_distance=5.0, max_angle_change=np.pi/4, max_borders=2):
#     """
#     Cluster cones into separate border segments based on distance and continuity.
#     Limits the maximum number of borders to max_borders.
    
#     Parameters:
#     - cones: List of cone points
#     - max_distance: Maximum distance between consecutive cones in a border
#     - max_angle_change: Maximum allowed angle change between consecutive segments
#     - max_borders: Maximum number of borders to generate
#     """
#     if not cones or len(cones) < 2:
#         return [cones] if cones else []
    
#     # Convert to numpy array for easier manipulation
#     cones_array = np.array(cones)
    
#     # Start with a single cone in the first border
#     borders = [[cones_array[0]]]
#     remaining_cones = cones_array[1:].tolist()
    
#     while remaining_cones:
#         current_border = borders[-1]
#         last_cone = current_border[-1]
        
#         # Calculate distances from last cone to all remaining cones
#         distances = [np.linalg.norm(np.array(last_cone[:2]) - np.array(cone[:2])) for cone in remaining_cones]
        
#         if min(distances) > max_distance:
#             # If all remaining cones are too far, start a new border (if we haven't reached max_borders)
#             if len(borders) < max_borders:
#                 borders.append([remaining_cones[0]])
#                 remaining_cones.pop(0)
#             else:
#                 # We've reached max_borders, find the closest cone to any existing border
#                 best_distance = float('inf')
#                 best_border_idx = 0
#                 best_cone_idx = 0
                
#                 for border_idx, border in enumerate(borders):
#                     for cone_idx, cone in enumerate(remaining_cones):
#                         for border_cone in border:
#                             dist = np.linalg.norm(np.array(border_cone[:2]) - np.array(cone[:2]))
#                             if dist < best_distance:
#                                 best_distance = dist
#                                 best_border_idx = border_idx
#                                 best_cone_idx = cone_idx
                
#                 # Add the best cone to the best border
#                 borders[best_border_idx].append(remaining_cones[best_cone_idx])
#                 remaining_cones.pop(best_cone_idx)
#             continue
        
#         # Find candidates within max_distance
#         candidates = []
#         for i, cone in enumerate(remaining_cones):
#             if distances[i] <= max_distance:
#                 candidates.append((i, cone))
        
#         # If we have at least 2 cones in the current border, check angle constraints
#         if len(current_border) >= 2:
#             # Calculate the direction vector of the current border
#             prev_cone = current_border[-2]
#             current_direction = np.array(last_cone[:2]) - np.array(prev_cone[:2])
#             current_direction = current_direction / np.linalg.norm(current_direction)
            
#             # Filter candidates by angle constraint
#             valid_candidates = []
#             for idx, cone in candidates:
#                 new_direction = np.array(cone[:2]) - np.array(last_cone[:2])
#                 new_direction = new_direction / np.linalg.norm(new_direction)
                
#                 # Calculate the angle between directions
#                 dot_product = np.dot(current_direction, new_direction)
#                 angle = np.arccos(np.clip(dot_product, -1.0, 1.0))
                
#                 # If angle is within the constraint, add to valid candidates
#                 if angle <= max_angle_change:
#                     valid_candidates.append((idx, cone, angle))
            
#             # If we have valid candidates, choose the one with the smallest angle change
#             if valid_candidates:
#                 valid_candidates.sort(key=lambda x: x[2])  # Sort by angle
#                 closest_idx, closest_cone, _ = valid_candidates[0]
#             else:
#                 # No valid candidates, start a new border if possible
#                 if len(borders) < max_borders:
#                     borders.append([remaining_cones[0]])
#                     remaining_cones.pop(0)
#                 else:
#                     # We've reached max_borders, add to the most appropriate existing border
#                     best_fit_border = 0
#                     min_angle_diff = float('inf')
#                     best_cone_idx = 0
                    
#                     for border_idx, border in enumerate(borders):
#                         if len(border) >= 2:
#                             border_direction = np.array(border[-1][:2]) - np.array(border[-2][:2])
#                             border_direction = border_direction / np.linalg.norm(border_direction)
                            
#                             for cone_idx, cone in enumerate(remaining_cones):
#                                 cone_direction = np.array(cone[:2]) - np.array(border[-1][:2])
#                                 if np.linalg.norm(cone_direction) > 0:  # Avoid division by zero
#                                     cone_direction = cone_direction / np.linalg.norm(cone_direction)
#                                     angle_diff = np.arccos(np.clip(np.dot(border_direction, cone_direction), -1.0, 1.0))
                                    
#                                     if angle_diff < min_angle_diff:
#                                         min_angle_diff = angle_diff
#                                         best_fit_border = border_idx
#                                         best_cone_idx = cone_idx
                    
#                     borders[best_fit_border].append(remaining_cones[best_cone_idx])
#                     remaining_cones.pop(best_cone_idx)
#                 continue
#         else:
#             # If we have only one cone in the border, choose the closest one
#             closest_idx = np.argmin(distances)
#             closest_cone = remaining_cones[closest_idx]
        
#         # Add the chosen cone to the current border
#         current_border.append(closest_cone)
#         remaining_cones.pop(closest_idx)
    
#     # If we have more than max_borders, merge the smallest ones
#     while len(borders) > max_borders:
#         # Find the two smallest borders
#         border_sizes = [len(border) for border in borders]
#         smallest_idx = border_sizes.index(min(border_sizes))
        
#         # Remove the smallest border from the list
#         smallest_border = borders.pop(smallest_idx)
        
#         # Find the closest border to merge with
#         if borders:
#             best_border_idx = 0
#             min_distance = float('inf')
            
#             for i, border in enumerate(borders):
#                 for cone1 in smallest_border:
#                     for cone2 in border:
#                         dist = np.linalg.norm(np.array(cone1[:2]) - np.array(cone2[:2]))
#                         if dist < min_distance:
#                             min_distance = dist
#                             best_border_idx = i
            
#             # Merge the smallest border with the closest one
#             borders[best_border_idx].extend(smallest_border)
    
#     # Sort each border by x-coordinate for final ordering
#     for i in range(len(borders)):
#         borders[i] = sorted(borders[i], key=lambda p: p[0])
    
#     return borders


# def group_cones_into_polylines(data_dict):
#     grouped_polylines = {}

#     for label_id, cones in data_dict.items():
#         grouped_polylines[label_id] = {'Left_Border': [], 'Right_Border': [], 'Start': []}

#         for class_name, points in cones.items():
#             if class_name == 'Cone_Blue':
#                 grouped_polylines[label_id]['Left_Border'].extend(points)
#             elif class_name == 'Cone_Yellow':
#                 grouped_polylines[label_id]['Right_Border'].extend(points)
#             elif class_name == 'Cone_Big':
#                 grouped_polylines[label_id]['Start'].extend(points)

#         # Sort points within each border by their x-coordinate to form polylines
#         for border in ['Left_Border', 'Right_Border', 'Start']:
#             grouped_polylines[label_id][border] = sorted(grouped_polylines[label_id][border], key=lambda p: p[0])

#     return grouped_polylines

def determine_number_of_borders(cones, max_distance=5.0):
    """
    Custom clustering algorithm to determine the number of borders (1 or 2).
    """
    if len(cones) < 2:
        return 1  # If there are fewer than 2 cones, assume 1 border

    # Sort cones by x-coordinate for initial ordering
    sorted_cones = sorted(cones, key=lambda p: np.linalg.norm(p[0]))

    # Initialize clusters
    clusters = [[sorted_cones[0]]]

    # Assign each cone to a cluster
    for cone in sorted_cones[1:]:
        # Check distance to the last cone in the current cluster
        last_cone = clusters[-1][-1]
        distance = np.linalg.norm(np.array(cone[:2]) - np.array(last_cone[:2]))

        if distance <= max_distance:
            clusters[-1].append(cone)
        else:
            clusters.append([cone])

    # Determine the number of borders based on the number of clusters
    return min(len(clusters), 2)  # Limit to a maximum of 2 borders

# def determine_number_of_borders(cones, max_distance=5.0):
#     """
#     Custom clustering algorithm to determine the number of borders (1 or 2).
#     First picks the first cone by the lowest x-coordinate within 7 meters,
#     then sorts cones using nearest-neighbor approach, and finally clusters them.
#     """
#     if len(cones) < 2:
#         return 1  # If there are fewer than 2 cones, assume 1 border

#     # Filter cones within 7 meters from the origin
#     cones_within_7m = [cone for cone in cones if np.linalg.norm(np.array(cone[:2])) <= 7.0]

#     if not cones_within_7m:
#         return 1  # If no cones are within 7 meters, assume 1 border

#     # Pick the first cone by the lowest x-coordinate
#     start_idx = np.argmin([cone[0] for cone in cones_within_7m])
#     current_cone = cones_within_7m.pop(start_idx)
#     sorted_cones = [current_cone]

#     # Continue picking the closest cone to the last added cone
#     remaining_cones = cones_within_7m.copy()
#     while remaining_cones:
#         distances = [np.linalg.norm(np.array(cone[:2]) - np.array(current_cone[:2])) for cone in remaining_cones]
#         closest_idx = np.argmin(distances)
#         current_cone = remaining_cones.pop(closest_idx)
#         sorted_cones.append(current_cone)

#     # Initialize clusters
#     clusters = [[sorted_cones[0]]]

#     # Assign each cone to a cluster
#     for cone in sorted_cones[1:]:
#         # Check distance to the last cone in the current cluster
#         last_cone = clusters[-1][-1]
#         distance = np.linalg.norm(np.array(cone[:2]) - np.array(last_cone[:2]))

#         if distance <= max_distance:
#             clusters[-1].append(cone)
#         else:
#             clusters.append([cone])

#     # Determine the number of borders based on the number of clusters
#     return min(len(clusters), 2)  # Limit to a maximum of 2 borders

def cluster_cones_into_borders(cones, max_distance=20.0, max_angle_change=np.pi/4):
    """
    Cluster cones into borders using spectral clustering with parameters optimized for track border detection.
    """
    if not cones or len(cones) < 3:  # Need at least 3 points for meaningful clustering
        return [cones] if cones else []
    
    # Convert to numpy array
    cones_array = np.array(cones)
    
    # Determine the number of borders
    num_borders = determine_number_of_borders(cones, max_distance)
    
    # Create affinity matrix based on distance and angle constraints
    n_samples = len(cones_array)
    affinity_matrix = np.zeros((n_samples, n_samples))
    
    for i in range(n_samples):
        for j in range(n_samples):
            if i == j:
                affinity_matrix[i, j] = 1.0  # Self-similarity is maximum
                continue
                
            # Calculate Euclidean distance between points (in xy plane)
            distance = np.linalg.norm(cones_array[i, :2] - cones_array[j, :2])
            
            # Only consider points within max_distance
            if distance <= max_distance:
                # Use a Gaussian kernel with adaptive bandwidth
                sigma = max_distance / 3  # Adaptive bandwidth
                affinity_matrix[i, j] = np.exp(-distance**2 / (2 * sigma**2))
    
    # Apply Spectral Clustering with custom affinity matrix
    spectral = SpectralClustering(
        n_clusters=num_borders,
        affinity='precomputed',#'nearest_neighbors',  # Use our custom affinity matrix
        eigen_solver='arpack',   # More stable for this application
        assign_labels='kmeans',  # KMeans for final clustering step
        n_init=100,               # More initializations for better results
        random_state=42
    )
    
    # Fit the model and predict cluster labels
    labels = spectral.fit_predict(affinity_matrix)
    
    # Group cones by cluster label
    borders = [[] for _ in range(num_borders)]
    for i, label in enumerate(labels):
        borders[label].append(cones_array[i])
    
    # Sort points within each border to form a continuous polyline
    for i in range(len(borders)):
        if len(borders[i]) > 2:
            # Use a greedy algorithm to sort points by proximity
            sorted_border = [borders[i][0]]  # Start with first point
            remaining = borders[i][1:]
            
            while remaining:
                last_point = sorted_border[-1]
                # Find closest remaining point
                distances = [np.linalg.norm(np.array(last_point[:2]) - np.array(p[:2])) for p in remaining]
                closest_idx = np.argmin(distances)
                
                # Add closest point to sorted list
                sorted_border.append(remaining[closest_idx])
                remaining.pop(closest_idx)
            
            borders[i] = sorted_border
    
    return borders


import open3d as o3d
import numpy as np

def visualize_borders(borders, colors=None, label_id=""):
    # Create visualization elements
    vis_elements = []
    
    # Create a different color for each border if not provided
    if colors is None:
        colors = [[1, 0, 0], [0, 1, 0]]  # Red for first border, green for second
    
    # Create line sets for each border
    for i, border in enumerate(borders):
        # Create points for the border
        points = np.array(border)
        
        # Create lines connecting consecutive points
        lines = [[j, j+1] for j in range(len(border)-1)]
        
        # Create line colors (using the border's assigned color)
        line_colors = [colors[i % len(colors)] for _ in range(len(lines))]
        
        # Create the line set
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector(line_colors)
        
        vis_elements.append(line_set)

    # # Use Visualizer to render and save the image
    # vis = o3d.visualization.Visualizer()
    # vis.create_window(visible=False)  # Set visible=False if you don't want to show the window
    # for element in vis_elements:
    #     vis.add_geometry(element)
    
    # vis.poll_events()
    # vis.update_renderer()
    
    # # Save the current screen to a PNG file
    # output_path = f"/home/dobayt/ros2_ws/src/formula_student_packages/lidar_centerpoint_detector/data_utils/o3d_pngs/{label_id}.png"
    # vis.capture_screen_image(output_path)
    # print(f"Visualization saved to {output_path}")
    
    # vis.destroy_window()
    
    # Visualize all borders together
    o3d.visualization.draw_geometries(vis_elements)

def write_polylines_to_txt(grouped_polylines, output_folder):
    output_folder = Path(output_folder)
    output_folder.mkdir(parents=True, exist_ok=True)

    for label_id, borders in grouped_polylines.items():
        output_file = output_folder / f"{label_id}.txt"
        with open(output_file, 'w') as file:
            for border, polylines in borders.items():
                # Flatten the polyline points and convert to string
                polyline_str = ' '.join(map(str, np.array(polylines).flatten()))
                file.write(f"{polyline_str} {border}\n")

def group_cones_into_borders(yellow_cones: list, blue_cones: list = [], label_id: str = '') -> list[list]:
    if yellow_cones and not blue_cones:
        # Determine the number of borders
        num_borders = determine_number_of_borders(yellow_cones, 5.0)
        # Convert to numpy array
        cones_array = np.array(yellow_cones)
        # sort by x coordinate
        cones_array = cones_array[cones_array[:, 0].argsort()]

        borders = []
        used_idx = []
        for i in range(num_borders):
            border = []
            if i > 0:
                # check if there are still available points
                if len(used_idx) == cones_array.shape[0]:
                    break
                step_back = 1
                for j in range(cones_array.shape[0]):
                    # check if all the points are already assigned
                    if len(used_idx) == cones_array.shape[0]:
                        break
                    if not border:
                        for k in range(cones_array.shape[0]):
                            if cones_array[k,1] >= 2.0 and k not in used_idx:
                                border.append(cones_array[k,:])
                                used_idx.append(k)
                                break
                    else:
                        prev_point_dist_to_other = np.linalg.norm(cones_array - border[j-step_back], axis=1)

                        # Filter excluded indices
                        valid_mask = np.ones(len(prev_point_dist_to_other), dtype=bool)
                        valid_mask[used_idx] = False
                        filtered_indices = np.where(valid_mask)[0]

                        closest_idx = filtered_indices[np.argmin(prev_point_dist_to_other[valid_mask])]
                        # first check if it is closer then the thr
                        if prev_point_dist_to_other[closest_idx] < 7.0:
                            border.append(cones_array[closest_idx,:])
                            used_idx.append(closest_idx)
                        # otherwise check which border is it closer to
                        else:
                            first_border_l2 = [np.linalg.norm(point) for point in borders[0]]
                            first_border_endpoint = borders[0][first_border_l2.index(max(first_border_l2))]
                            dist_to_other_border_end = np.linalg.norm(first_border_endpoint - cones_array[closest_idx,:])
                            if prev_point_dist_to_other[closest_idx] > dist_to_other_border_end:
                                borders[0].append(cones_array[closest_idx,:])
                                # increment index decrease, bc border didn't got larger in this cycle
                                step_back += 1
                                used_idx.append(closest_idx)
                            else:
                                border.append(cones_array[closest_idx,:])
                                used_idx.append(closest_idx)

                # valid_mask = np.ones(cones_array.shape[0], dtype=bool)
                # valid_mask[used_idx] = False
                # border = [point for point in cones_array[valid_mask]]
                # borders.append(border)
                # break
            else:
                for j in range(cones_array.shape[0]):
                    if not border:
                        # check which is the first 3d point with negative y coordinate
                        for k in range(cones_array.shape[0]):
                            if cones_array[k,1] < 2.0 and np.linalg.norm(cones_array[k,1]) < 5.0:
                                border.append(cones_array[k,:])
                                used_idx.append(k)
                                break
                    else:
                        prev_point_dist_to_other = np.linalg.norm(cones_array - border[j-1], axis=1)

                        # Filter excluded indices
                        valid_mask = np.ones(len(prev_point_dist_to_other), dtype=bool)
                        valid_mask[used_idx] = False
                        filtered_indices = np.where(valid_mask)[0]
                        if len(used_idx) == 7:
                            a=0

                        closest_idx = filtered_indices[np.argmin(prev_point_dist_to_other[valid_mask])]
                        if prev_point_dist_to_other[closest_idx] < 7.0:
                            border.append(cones_array[closest_idx,:])
                            used_idx.append(closest_idx)
                        else:
                            break
            borders.append(border)

    else:
        # Determine the number of borders
        #num_borders = determine_number_of_borders(blue_cones, 3.5)
        if (int(label_id) > 0 and int(label_id) < 103) or (int(label_id) > 265 and int(label_id) < 391) or (int(label_id) > 547 and int(label_id) < 622):
            num_borders = 2
        else:
            num_borders = 1
        # Convert to numpy array
        cones_array_yellow = np.array(yellow_cones)
        cones_array_blue = np.array(blue_cones)
        # sort by x coordinate
        cones_array_yellow = cones_array_yellow[cones_array_yellow[:, 0].argsort()]
        cones_array_blue = cones_array_blue[cones_array_blue[:, 0].argsort()]

        borders = []
        used_idx = []
        if num_borders > 1:
            for i in range(num_borders):
                border = []
                if i > 0:
                    # check if there are still available points
                    if len(used_idx) == cones_array_blue.shape[0]:
                        break
                    step_back = 1
                    for j in range(cones_array_blue.shape[0]):
                        # check if all the points are already assigned
                        if len(used_idx) == cones_array_blue.shape[0]:
                            break
                        if not border:
                            for k in range(cones_array_blue.shape[0]):
                                if cones_array_blue[k,1] >= 3.0 and k not in used_idx:
                                    border.append(cones_array_blue[k,:])
                                    used_idx.append(k)
                                    break
                        else:
                            prev_point_dist_to_other = np.linalg.norm(cones_array_blue - border[j-step_back], axis=1)

                            # Filter excluded indices
                            valid_mask = np.ones(len(prev_point_dist_to_other), dtype=bool)
                            valid_mask[used_idx] = False
                            filtered_indices = np.where(valid_mask)[0]
                            if len(used_idx) == 23 and int(label_id) == 65:
                                a = 0
                            if prev_point_dist_to_other[valid_mask].shape[0] == 1:
                                closest_idx = filtered_indices[np.argmin(prev_point_dist_to_other[valid_mask])]
                                prev_point_dist_to_other_border = np.linalg.norm(cones_array_blue[closest_idx,:] - borders[0][-1])
                                angle_with_border_last_point = abs(np.arctan2(cones_array_blue[closest_idx,1] - border[-1][1], cones_array_blue[closest_idx,0] - border[-1][0]) * 180 / np.pi)
                                angle_with_other_border_last_point = abs(np.arctan2(cones_array_blue[closest_idx,1] - borders[0][-1][1], cones_array_blue[closest_idx,0] - borders[0][-1][0]) * 180 / np.pi)
                                #if prev_point_dist_to_other[closest_idx] < 7.5 and prev_point_dist_to_other_border > 12:
                                if prev_point_dist_to_other[closest_idx] < prev_point_dist_to_other_border and abs(prev_point_dist_to_other[closest_idx] - prev_point_dist_to_other_border) > 1.5 and abs(angle_with_border_last_point - angle_with_other_border_last_point) < 60:
                                    border.append(cones_array_blue[closest_idx,:])
                                    used_idx.append(closest_idx)
                                elif prev_point_dist_to_other[closest_idx] < 7.5 and angle_with_border_last_point < angle_with_other_border_last_point:
                                    border.append(cones_array_blue[closest_idx,:])
                                    used_idx.append(closest_idx)
                                elif abs(prev_point_dist_to_other[closest_idx] - prev_point_dist_to_other_border) < 0.6 and angle_with_border_last_point < angle_with_other_border_last_point:
                                    border.append(cones_array_blue[closest_idx,:])
                                    used_idx.append(closest_idx)
                                else: 
                                    borders[0].append(cones_array_blue[closest_idx,:])
                                    step_back += 1
                                    used_idx.append(closest_idx)
                                # MODIFY LOGIC
                                # else:
                                #     first_border_l2 = [np.linalg.norm(point) for point in borders[0]]
                                #     first_border_endpoint = borders[0][first_border_l2.index(max(first_border_l2))]
                                #     dist_to_other_border_end = np.linalg.norm(first_border_endpoint - cones_array_blue[closest_idx,:])
                                #     if prev_point_dist_to_other[closest_idx] > dist_to_other_border_end:
                                #         borders[0].append(cones_array_blue[closest_idx,:])
                                #         # increment index decrease, bc border didn't got larger in this cycle
                                #         step_back += 1
                                #         used_idx.append(closest_idx)
                                #     else:
                                #         border.append(cones_array_blue[closest_idx,:])
                                #         used_idx.append(closest_idx)
                            else:
                                if len(prev_point_dist_to_other[valid_mask].argsort()) > 2:
                                    closest_idx_top3 = filtered_indices[prev_point_dist_to_other[valid_mask].argsort()[:3]]
                                    top3 = True
                                else:
                                    closest_idx_top3 = filtered_indices[prev_point_dist_to_other[valid_mask].argsort()[:2]]
                                    top3 = False
                                # first check if it is closer then the thr
                                if prev_point_dist_to_other[closest_idx_top3[0]] < 7.5 and prev_point_dist_to_other[closest_idx_top3[1]] < 7.5:
                                    if len(border) == 1:
                                        previous_angle_deg = 9999999
                                        for idx in closest_idx_top3:
                                            angle_with_prev_border_point = abs(np.arctan2(cones_array_blue[idx,1] - border[j-1][1], cones_array_blue[idx,0] - border[j-1][0]) * 180 / np.pi)
                                            if angle_with_prev_border_point < previous_angle_deg and abs(angle_with_prev_border_point - previous_angle_deg) > 8:
                                                previous_angle_deg = angle_with_prev_border_point
                                                closest_idx = idx
                                        border.append(cones_array_blue[closest_idx,:])
                                        used_idx.append(closest_idx)
                                    else:
                                        first_border_l2 = [np.linalg.norm(point) for point in borders[0]]
                                        first_border_endpoint = borders[0][first_border_l2.index(max(first_border_l2))]
                                        dist_to_other_border_end_first_closest = np.linalg.norm(first_border_endpoint - cones_array_blue[closest_idx_top3[0],:])
                                        dist_to_other_border_end_second_closest = np.linalg.norm(first_border_endpoint - cones_array_blue[closest_idx_top3[1],:])
                                        
                                        angle_with_prev_border_point_first_closest = np.arctan2(cones_array_blue[closest_idx_top3[0],1] - border[j-1][1], cones_array_blue[closest_idx_top3[0],0] - border[j-1][0]) * 180 / np.pi
                                        angle_with_prev_border_point_second_closest = np.arctan2(cones_array_blue[closest_idx_top3[1],1] - border[j-1][1], cones_array_blue[closest_idx_top3[1],0] - border[j-1][0]) * 180 / np.pi
                                        if top3:
                                            angle_with_prev_border_point_third_closest = np.arctan2(cones_array_blue[closest_idx_top3[2],1] - border[j-1][1], cones_array_blue[closest_idx_top3[2],0] - border[j-1][0]) * 180 / np.pi
                                        angle_prev_border_segment = np.arctan2(border[j-1][1] - border[j-2][1], border[j-1][0] - border[j-2][0]) * 180 / np.pi
                                        # If the angle difference is not large with the last border point, prioritize distance to the current border
                                        if abs(angle_with_prev_border_point_first_closest - angle_with_prev_border_point_second_closest) < 20:
                                            border.append(cones_array_blue[closest_idx_top3[0],:])
                                            used_idx.append(closest_idx_top3[0])
                                        else:
                                            # If the angle difference is large of the two closest point compared to last border point, prioritize distance to other border
                                            if abs(dist_to_other_border_end_first_closest - dist_to_other_border_end_second_closest) < 3.12 and abs(angle_prev_border_segment - angle_with_prev_border_point_first_closest) < 60:
                                                border.append(cones_array_blue[closest_idx_top3[0],:])
                                                used_idx.append(closest_idx_top3[0])
                                            elif dist_to_other_border_end_first_closest > dist_to_other_border_end_second_closest and abs(angle_prev_border_segment - angle_with_prev_border_point_first_closest) < 60:
                                                border.append(cones_array_blue[closest_idx_top3[0],:])
                                                used_idx.append(closest_idx_top3[0])
                                            elif abs(angle_prev_border_segment - angle_with_prev_border_point_second_closest) < 60:
                                                border.append(cones_array_blue[closest_idx_top3[1],:])
                                                used_idx.append(closest_idx_top3[1])
                                            elif top3 and abs(angle_prev_border_segment - angle_with_prev_border_point_third_closest) < 60:
                                                border.append(cones_array_blue[closest_idx_top3[2],:])
                                                used_idx.append(closest_idx_top3[2])
                                            else:
                                                borders[0].extend([cone for cone in cones_array_blue[valid_mask,:]])
                                                break

                                    # border.append(cones_array_blue[closest_idx,:])
                                    # used_idx.append(closest_idx)
                                elif prev_point_dist_to_other[closest_idx_top3[0]] < 7.5 and not prev_point_dist_to_other[closest_idx_top3[1]] < 7.5:
                                    angle_with_prev_border_point_first_closest = np.arctan2(cones_array_blue[closest_idx_top3[0],1] - border[j-1][1], cones_array_blue[closest_idx_top3[0],0] - border[j-1][0]) * 180 / np.pi
                                    angle_with_prev_border_point_second_closest = np.arctan2(cones_array_blue[closest_idx_top3[1],1] - border[j-1][1], cones_array_blue[closest_idx_top3[1],0] - border[j-1][0]) * 180 / np.pi
                                    angle_prev_border_segment = np.arctan2(border[j-1][1] - border[j-2][1], border[j-1][0] - border[j-2][0]) * 180 / np.pi
                                    if not top3 and abs(angle_prev_border_segment - angle_with_prev_border_point_first_closest) > abs(angle_prev_border_segment - angle_with_prev_border_point_second_closest):
                                        border.append(cones_array_blue[closest_idx_top3[1],:])
                                        used_idx.append(closest_idx_top3[1])
                                    else:
                                        border.append(cones_array_blue[closest_idx_top3[0],:])
                                        used_idx.append(closest_idx_top3[0])
                                elif not prev_point_dist_to_other[closest_idx_top3[0]] < 7.5 and prev_point_dist_to_other[closest_idx_top3[1]] < 7.5:
                                    border.append(cones_array_blue[closest_idx_top3[1],:])
                                    used_idx.append(closest_idx_top3[1])
                                # otherwise check which border is it closer to
                                else:
                                    first_border_l2 = [np.linalg.norm(point) for point in borders[0]]
                                    first_border_endpoint = borders[0][first_border_l2.index(max(first_border_l2))]
                                    dist_to_other_border_end = np.linalg.norm(first_border_endpoint - cones_array_blue[closest_idx,:])
                                    if prev_point_dist_to_other[closest_idx] > dist_to_other_border_end:
                                        borders[0].append(cones_array_blue[closest_idx,:])
                                        # increment index decrease, bc border didn't got larger in this cycle
                                        step_back += 1
                                        used_idx.append(closest_idx)
                                    else:
                                        border.append(cones_array_blue[closest_idx,:])
                                        used_idx.append(closest_idx)


                    # valid_mask = np.ones(cones_array.shape[0], dtype=bool)
                    # valid_mask[used_idx] = False
                    # border = [point for point in cones_array[valid_mask]]
                    # borders.append(border)
                    # break
                else:
                    for j in range(cones_array_blue.shape[0]):
                        if not border:
                            if int(label_id) == 55:
                                a = 0
                            # check which is the first 3d point with negative y coordinate
                            for k in range(cones_array_blue.shape[0]):
                                if cones_array_blue[k,1] > 1.7 and np.linalg.norm(cones_array_blue[k,:]) < 6.5 and cones_array_blue[k,1] < 3.4:
                                    border.append(cones_array_blue[k,:])
                                    used_idx.append(k)
                                    break
                        else:
                            prev_point_dist_to_other = np.linalg.norm(cones_array_blue - border[j-1], axis=1)

                            # Filter excluded indices
                            valid_mask = np.ones(len(prev_point_dist_to_other), dtype=bool)
                            valid_mask[used_idx] = False
                            filtered_indices = np.where(valid_mask)[0]
                            if len(used_idx) == 2 and int(label_id) == 266:
                                a=0

                            closest_idx_top4 = filtered_indices[prev_point_dist_to_other[valid_mask].argsort()[:4]]

                            if abs(np.linalg.norm(cones_array_blue[closest_idx_top4[0],:] - border[j-1]) - np.linalg.norm(cones_array_blue[closest_idx_top4[1],:] - border[j-1])) > 2.5 and abs(np.arctan2(cones_array_blue[closest_idx_top4[0],1] - border[j-1][1], cones_array_blue[closest_idx_top4[0],0] - border[j-1][0]) * 180 / np.pi) < 70:
                                closest_idx = closest_idx_top4[0]
                            elif len(border) == 1:
                                previous_angle_deg = 9999999
                                for idx in closest_idx_top4:
                                    angle_with_prev_border_point = abs(np.arctan2(cones_array_blue[idx,1] - border[j-1][1], cones_array_blue[idx,0] - border[j-1][0]) * 180 / np.pi)
                                    if angle_with_prev_border_point < previous_angle_deg and abs(angle_with_prev_border_point - previous_angle_deg) > 3:
                                        previous_angle_deg = angle_with_prev_border_point
                                        closest_idx = idx
                            else:
                                previous_angle_deg_border_points = abs(np.arctan2(border[j-1][1] - border[j-2][1], border[j-1][0] - border[j-2][0]) * 180 / np.pi)
                                previous_angle_diff = 999999
                                for idx in closest_idx_top4:
                                    angle_with_prev_border_point = abs(np.arctan2(cones_array_blue[idx,1] - border[j-1][1], cones_array_blue[idx,0] - border[j-1][0]) * 180 / np.pi)
                                    if abs(angle_with_prev_border_point - previous_angle_deg_border_points) < previous_angle_diff and abs(angle_with_prev_border_point - previous_angle_deg_border_points) <= 30.0 and abs(abs(angle_with_prev_border_point - previous_angle_deg_border_points) - previous_angle_diff) > 3:
                                        previous_angle_diff = abs(angle_with_prev_border_point - previous_angle_deg_border_points)
                                        closest_idx = idx
                                if previous_angle_diff == 999999:
                                    break

                            if prev_point_dist_to_other[closest_idx] < 7.5:
                                border.append(cones_array_blue[closest_idx,:])
                                used_idx.append(closest_idx)
                            else:
                                break
                borders.append(border)
        else:
            remaining_cones = cones_array_blue.copy()
            # convert it to list
            remaining_cones = [remaining_cones[i,:] for i in range(remaining_cones.shape[0])]
            for k in range(cones_array_blue.shape[0]):
                if cones_array_blue[k,1] > 1.7 and np.linalg.norm(cones_array_blue[k,:]) < 6.5 and cones_array_blue[k,1] < 4.0:
                    current_cone = cones_array_blue[k,:]
                    sorted_cones = [current_cone]
                    remaining_cones.pop(k)
                    break

            while remaining_cones:
                if len(remaining_cones) == 19:
                    a = 0
                distances = [np.linalg.norm(np.array(cone) - np.array(current_cone)) for cone in remaining_cones]
                closest_idx_top4 = np.array(distances).argsort()[:3]
                previous_angle_deg = 9999999
                for idx in closest_idx_top4:
                    if len(sorted_cones) == 1:
                        angle_with_current_point = abs(np.arctan2(remaining_cones[idx][1] - current_cone[1], remaining_cones[idx][0] - current_cone[0]) * 180 / np.pi)
                        if angle_with_current_point < previous_angle_deg and abs(angle_with_current_point - previous_angle_deg) > 10:
                            previous_angle_deg = angle_with_current_point
                            closest_idx = idx
                    else:
                        angle_with_current_point = abs(np.arctan2(remaining_cones[idx][1] - current_cone[1], remaining_cones[idx][0] - current_cone[0]) * 180 / np.pi)
                        angle_closest_segment = abs(np.arctan2(sorted_cones[-1][1] - sorted_cones[-2][1], sorted_cones[-1][0] - sorted_cones[-2][0]) * 180 / np.pi)
                        if angle_with_current_point < previous_angle_deg and abs(angle_with_current_point - previous_angle_deg) > 10 and abs(angle_closest_segment - angle_with_current_point) < 40:
                            previous_angle_deg = angle_with_current_point
                            closest_idx = idx
                    if previous_angle_deg == 9999999:
                        closest_idx = closest_idx_top4[0]
                current_cone = remaining_cones.pop(closest_idx)
                sorted_cones.append(current_cone)
            borders.append(sorted_cones)

    return borders

def create_polyline_label(data_dict, output_folder):
    grouped_polylines = {}
    
    for label_id, cones in data_dict.items():
        grouped_polylines[label_id] = {}
        if int(label_id) < 251:
            # Process yellow cones (Right Border)
            if 'Cone_Yellow' in cones:
                yellow_cones = cones['Cone_Yellow']
                right_borders = group_cones_into_borders(yellow_cones, label_id=label_id)
                # if int(label_id) > 99999:
                #     visualize_borders(right_borders, [[0, 0, 1], [0, 1, 1]], label_id)
                for i, border in enumerate(right_borders):
                    grouped_polylines[label_id][f'Right_Border_{i+1}'] = border
            
            # Process blue cones (Left Border)
            if 'Cone_Blue' in cones:
                blue_cones = cones['Cone_Blue']
                if int(label_id) == 266:
                    a = 0
                left_borders = group_cones_into_borders(yellow_cones, blue_cones, label_id)
                #left_borders = cluster_cones_into_borders(blue_cones, max_distance=5.0, max_angle_change=np.pi/4)
                # if int(label_id) > 265 and int(label_id) < 391:
                #     visualize_borders(left_borders, [[0, 0, 1], [0, 1, 1]])
                for i, border in enumerate(left_borders):
                    grouped_polylines[label_id][f'Left_Border_{i+1}'] = border
            
            # # Process big cones (Start)
            # if 'Cone_Big' in cones:
            #     grouped_polylines[label_id]['Start'] = cones['Cone_Big']
    
    # Write the grouped polylines to txt files
    write_polylines_to_txt(grouped_polylines, output_folder)
    
    return grouped_polylines



def visualize_labels_2d():
    pass

def create_data_dict(input_folder: Path, ref_point: str, transform: bool) -> dict:
    data_dict = {}
    if transform:
        metadata_file = input_folder.parent / "metadata.json"
        with open(metadata_file, 'r') as file:
            metadata_dict = json.load(file)['data']
    for i, label_file in enumerate(input_folder.iterdir()):
        label_id = label_file.stem
        data_dict[label_id] = {}
        with open(label_file, 'r') as file:
            for line in file:
                # Split each line by spaces or tabs
                values = line.split()
                xyz = np.array(values[:3], dtype = np.float32)
                if transform:
                    H = np.eye(4, dtype=np.float32)
                    H[:3, :3] = Rotation.from_euler("xyz", [0.0, 0.0, metadata_dict[i]['odom']['yaw']]).as_matrix()
                    H[:3, 3] = [metadata_dict[i]['odom']['x'], metadata_dict[i]['odom']['y'], metadata_dict[i]['odom']['z']]
                    H = np.linalg.inv(H)
                    bbox_center = np.hstack((np.array([values[:3]], dtype=np.float32), np.eye(1, dtype=np.float32)))
                    bbox_center_tf = bbox_center @ H.T
                    xyz = bbox_center_tf[0][:3]
                # translate z coordinate if specified
                if ref_point == "top":
                    xyz[2] = xyz[2] + (float(values[5]) / 2)
                elif ref_point == "bottom":
                    xyz[2] = xyz[2] - (float(values[5]) / 2)
                # write point to data_dict
                if values[-1] not in data_dict[label_id].keys():
                    data_dict[label_id][values[-1]] = [xyz]
                else:
                    data_dict[label_id][values[-1]].append(xyz)
    return data_dict

def main(input_folder: Path, ref_point: str, visualize: bool, transform: bool) -> None:
    data_dict = create_data_dict(input_folder, ref_point, transform)

    # Create output folder
    output_folder = input_folder.parent / "polyline_labels"
    
    # Create polyline labels
    grouped_polylines = create_polyline_label(data_dict, output_folder)
    
    # Visualize if requested
    if visualize:
        visualize_labels_2d(grouped_polylines, output_folder)

    

if __name__ == "__main__":
    '''
    Example launch config:
        {
            "name": "polyline converter",
            "type": "python",
            "request": "launch",
            "program": "src/formula_student_packages/lidar_centerpoint_detector/data_utils/convert_bbox_to_polyline.py",
            "console": "integratedTerminal",
            "justMyCode": false,
            "args": [
                "/mnt/d/datasets/new_cfs/cfs_vargarda8/labels",
                "--transform"
            ]
        }
    '''
    parser = argparse.ArgumentParser(description="Convert 3D bbox labels to 3D polylines."
    "Accepts labels in format of txt files, where each row contains [x, y, z, l, w, h, yaw, class_name]"
    "Outputs polyline labels in same txt structure, where each row contains [x1, y1, z1, x2, y2, z2, ..., xn, yn, zn, class_name]")
    parser.add_argument(
        "input", type = str, help = "Input folder, where the label .txt files are present"
    )
    parser.add_argument(
        "--ref-point", default = "top", choices=["top", "center", "bottom"], required = False, type = str,
        help = "In the labels, the bbox center points are present. "
        "By default, we want the polyline points to be aligned with the top of the boxes, so translation is required."
        "Setting the argument to center will result in no translation."
    )
    parser.add_argument(
        "--transform", action="store_true", required = False,
        help = "Transform labels to local coordinate system, if Segments.ai annotation was done with odometry."
    )
    parser.add_argument(
        "--visualize", action="store_true", required = False,
        help = "Visualize the generated labels in a video."
    )
    
    args = parser.parse_args()
    main(Path(args.input), args.ref_point, args.visualize, args.transform)