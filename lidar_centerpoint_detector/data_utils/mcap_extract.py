# <--- full script with TF-from-mcap integration --->

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
import numpy as np
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from scipy.spatial.transform import Rotation as rot
import os
import sys
import shutil
import datetime as dt
import getpass as gt
import argparse
import json
import hashlib

reqdirs = ["unlabeled_pc", "points", "labels", "ImageSets"]
datasets = ["train.txt", "val.txt", "test.txt"]
ratio = [1, 1, 1]
pcl_fields = {
    'x' : 'x',
    'y' : 'y',
    'z' : 'z',
    'i' : 'intensity',
    'r' : 'reflectivity',
    'a' : 'ambient'
}

meta_dict = {
    "info": {
        "tool": "mcap_extract.py",
        "version": "0.3.2",
        "description": "Generated from MCAP file",
        "generated_on": dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "last_updated": dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "source": "source unknown",
        "user": "user unknown"
    },
    "data": [],
}

def manage_dirs(dir_out: str):
    print("Checking required directories...")
    for rdir in reqdirs:
        dpath = dir_out + "/" + rdir
        if os.path.isdir(dpath):
            print(f"Directory '{dpath}' exists.")
        else:
            os.makedirs(dpath)
            if os.path.isdir(dpath): print(f"Directory '{dpath}' created.")
            else: print("Failed to create: ", dpath)
    print("...done.")

def transform_points(pcl: np.array, H: np.array, D: int) -> np.array:
    """
    Transforms a 3D point cloud containing non-spatial attribute values using a homogeneous transformation matrix.

    Parameters:
        pcl (np.ndarray): An NxD (D>=3) array representing the 3D point cloud (x, y, z, ... [e.g. intensity]).
        H (np.ndarray): A 4x4 homogeneous transformation matrix.
        D: number of dimensions (attributes)

    Returns:
        np.ndarray: The transformed 3D point cloud with the non-spatial values preserved.
    """
    if pcl.size == 0:
        return pcl
    non_xyz = pcl[:, 3:].reshape(-1, D-3) if D > 3 else np.zeros((pcl.shape[0], 0), dtype=pcl.dtype)
    pcl_xyz = pcl[:, :3]
    pcl_h = np.hstack((pcl_xyz, np.ones((pcl_xyz.shape[0],1), dtype=pcl.dtype)))
    tranformed_pcl = pcl_h @ H.T
    tranformed_pcl = tranformed_pcl[:,:3]
    if D > 3:
        tranformed_pcl = np.hstack((tranformed_pcl, non_xyz))
    return tranformed_pcl

def getyaw(o):
    x, y, z, w = o.x, o.y, o.z, o.w
    return rot.from_quat([x, y, z, w]).as_euler("xyz")[2]

def extract_odom(msg, is_ps, msg0 = None): # 3rd arg optional, without: return absolute odom, with: subtract first odom (store if non-existent)
    if is_ps: # "PoseStamped" data type
        if msg0 is not None and msg0: # exists and not first (initialized/not empty)
            return {
            "timestamp": (msg.header.stamp.sec + msg.header.stamp.nanosec  * 1e-9 ) - msg0["timestamp"],
            "x": msg.pose.position.x - msg0["x"],
            "y": msg.pose.position.y - msg0["y"],
            "z": msg.pose.position.z - msg0["z"],
            "yaw": getyaw(msg.pose.orientation) - msg0["yaw"],
            "vx": 0.0,
            "vy": 0.0,
            "yawrate": 0.0
            }
        else: # first odom or "return absolute" mode
            odom = {
            "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 ,
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "z": msg.pose.position.z,
            "yaw": getyaw(msg.pose.orientation),
            "vx": 0.0,
            "vy": 0.0,
            "yawrate": 0.0
            }
            if msg0 is None:
                return odom # return as absolute value
            else: # if first msg
                msg0.update(odom) # store the absolute value to subtract later every time
                return {
                "timestamp": 0.0,
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "yaw": 0.0,
                "vx": 0.0,
                "vy": 0.0,
                "yawrate": 0.0
                }
    else: # "Odometry" data type
        if msg0 is not None and msg0: # exists and not first (initialized/not empty)
            return {
            "timestamp": (msg.header.stamp.sec+ msg.header.stamp.nanosec * 1e-9 ) - msg0["timestamp"],
            "x": msg.pose.pose.position.x - msg0["x"],
            "y": msg.pose.pose.position.y - msg0["y"],
            "z": msg.pose.pose.position.z - msg0["z"],
            "yaw": getyaw(msg.pose.pose.orientation) - msg0["yaw"],
            "vx": msg.twist.twist.linear.x - msg0["vx"],
            "vy": msg.twist.twist.linear.y - msg0["vy"],
            "yawrate": msg.twist.twist.angular.z - msg0["yawrate"]
            }
        else: # first odom or "return absolute" mode
            odom = {
            "timestamp": msg.header.stamp.sec+ msg.header.stamp.nanosec * 1e-9,
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "z": msg.pose.pose.position.z,
            "yaw": getyaw(msg.pose.pose.orientation),
            "vx": msg.twist.twist.linear.x,
            "vy": msg.twist.twist.linear.y,
            "yawrate": msg.twist.twist.angular.z
            }
            if msg0 is None:
                return odom # return as absolute value
            else: # if first msg
                msg0.update(odom) # store the absolute value to subtract later every time
                return {
                "timestamp": 0.0,
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "yaw": 0.0,
                "vx": 0.0,
                "vy": 0.0,
                "yawrate": 0.0
                }

# def get_closest_stamp_id(stamp: float, stamps: list): # todo: optimize
#     if not len(stamps): return -1
#     cid = 0
#     for i in range(0, len(stamps)):
#         current = stamp - stamps[i]["timestamp"]
#         if current <= 0.0:
#             if i == 0:
#                 cid = 0
#             else:
#                 if abs(current) > abs(stamp - stamps[i-1]["timestamp"]):
#                     cid = i-1
#                 else:
#                     cid = i
#             break
#     else:
#         cid = len(stamps)-1
#     return cid

def get_closest_stamp_id(stamp: int, stamps: list):
    """
    Find the index of the transform whose timestamp is closest to `stamp`.

    Assumes `stamps` is sorted ascending by "timestamp".
    `stamp` and `stamps[i]["timestamp"]` must be in the same time units
    (e.g., both nanoseconds or both seconds).
    """
    if not stamps:
        return -1

    closest_idx = 0
    closest_diff = abs(stamp - stamps[0]["timestamp"])

    for i in range(1, len(stamps)):
        diff = abs(stamp - stamps[i]["timestamp"])
        if diff < closest_diff:
            closest_diff = diff
            closest_idx = i
        else:
            # Since the list is sorted, once diff starts increasing we can stop
            break

    return closest_idx


import math

def get_interpolated_odom(stamp: float, odom_data: list, max_diff_threshold_ns=None, interpolate=True):
    """
    Find interpolated odometry data for a given timestamp (in nanoseconds).
    
    Args:
        stamp: Target timestamp in nanoseconds
        odom_data: List of odometry dicts [{"timestamp": ns, "x": ..., "vx": ...}]
        max_diff_threshold_ns: Max allowed time difference in nanoseconds
        interpolate: Whether to interpolate between readings
    
    Returns:
        Interpolated odometry dict or None if no valid match
    """
    if not odom_data:
        return None

    # Convert to numpy for vectorized operations
    timestamps = np.array([o["timestamp"] for o in odom_data])
    
    # Find insertion index (uses binary search for efficiency)
    idx = np.searchsorted(timestamps, stamp)
    
    # Handle edge cases
    if idx == 0:
        if len(odom_data) == 1:
            closest = odom_data[0]
        else:
            left = odom_data[0]
            right = odom_data[1]
    elif idx >= len(odom_data):
        left = odom_data[-2] if len(odom_data) > 1 else odom_data[-1]
        right = odom_data[-1]
    else:
        left = odom_data[idx-1]
        right = odom_data[idx]

    # Exact match check (with 1ms tolerance)
    if abs(left["timestamp"] - stamp) <= 1e6:  # 1ms in ns
        return left
    if abs(right["timestamp"] - stamp) <= 1e6:
        return right

    if not interpolate:
        # Return nearest neighbor
        closest = left if abs(left["timestamp"] - stamp) < abs(right["timestamp"] - stamp) else right
        if max_diff_threshold_ns and abs(closest["timestamp"] - stamp) > max_diff_threshold_ns:
            return None
        return closest

    # Full interpolation
    time_diff = right["timestamp"] - left["timestamp"]
    if time_diff == 0:  # Prevent division by zero
        return left
    
    ratio = (stamp - left["timestamp"]) / time_diff

    # Linear interpolation for position and velocity
    interp_odom = {
        "timestamp": stamp,
        "x": left["x"] + ratio * (right["x"] - left["x"]),
        "y": left["y"] + ratio * (right["y"] - left["y"]),
        "z": left["z"] + ratio * (right["z"] - left["z"]),
        "vx": left["vx"] + ratio * (right["vx"] - left["vx"]),
        "vy": left["vy"] + ratio * (right["vy"] - left["vy"]),
        "yawrate": left["yawrate"] + ratio * (right["yawrate"] - left["yawrate"])
    }

    # Angular interpolation for yaw (handles wrap-around)
    yaw_diff = ((right["yaw"] - left["yaw"] + np.pi) % (2*np.pi)) - np.pi
    interp_odom["yaw"] = left["yaw"] + ratio * yaw_diff

    if max_diff_threshold_ns:
        max_diff = max(abs(left["timestamp"] - stamp), abs(right["timestamp"] - stamp))
        if max_diff > max_diff_threshold_ns:
            return None

    return interp_odom


def save_meta(meta_dict, dir_out):
    with open(f"{dir_out}/metadata.json", "w") as f:
        meta_dict["info"]["last_updated"] = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        json.dump(meta_dict, f, indent = 4)

def init_reader(input_bag: str, pcl_topic: str, odom_topic: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    pcl_count = 0
    odom_count = 0
    for i in reader.get_metadata().topics_with_message_count:
        if i.topic_metadata.name == pcl_topic:
            pcl_count = i.message_count
            if odom_count: break
        if i.topic_metadata.name == odom_topic:
            odom_count = i.message_count
            if pcl_count: break
    return reader, pcl_count, odom_count, reader.get_metadata().message_count

# Modified read_messages: accept a list of topics to filter (we'll include /tf and /tf_static)
def read_messages(reader, topics):
    topic_types = reader.get_all_topics_and_types()
    def typename(topic_name):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic in topics:
            msg_type = get_message(typename(topic))
            msg = deserialize_message(data, msg_type)
            yield topic, msg, timestamp
    del reader

# helper: build homogeneous matrix from a geometry_msgs.msg.Transform (transform.translation, transform.rotation)
def transform_to_matrix(transform):
    tx = transform.translation.x
    ty = transform.translation.y
    tz = transform.translation.z
    qx = transform.rotation.x
    qy = transform.rotation.y
    qz = transform.rotation.z
    qw = transform.rotation.w
    Rm = rot.from_quat([qx, qy, qz, qw]).as_matrix()
    H = np.eye(4, dtype=np.float64)
    H[:3, :3] = Rm
    H[:3, 3] = [tx, ty, tz]
    return H

def get_closest_tf_matrix(pc_time, tf_list):
    """
    tf_list: list of dicts {'timestamp': float, 'H': np.array}
    returns H (4x4) closest in time, or identity if none
    """
    if not tf_list:
        return np.eye(4, dtype=np.float64)
    # build timestamps array once (tf_list is expected to be small enough)
    ts = np.array([t['timestamp'] for t in tf_list])
    idx = np.searchsorted(ts, pc_time)
    if idx == 0:
        return tf_list[0]['H']
    if idx >= len(ts):
        return tf_list[-1]['H']
    # pick closer of idx-1 and idx
    if abs(pc_time - ts[idx-1]) <= abs(pc_time - ts[idx]):
        return tf_list[idx-1]['H']
    else:
        return tf_list[idx]['H']

def main():
    print("Welcome to the annotation preprocessor! (MCAP to bin + folder structure + metadata.json)")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "file_in", type = str, help = "input bag path (folder or filepath) to read from"
    )
    parser.add_argument(
        "pcl_in", type = str, help = "name of the pointcloud to use"
    )
    parser.add_argument(
        "odom_in", type = str, help = "name of the odom topic to use"
    )
    parser.add_argument(
        "--fields", required = False, default = 'xyzi', type = str,
        help = "[optional] fields of the pointcloud - default = xyzi"
    )
    parser.add_argument(
        "--dir_out", default = "", required = False, type = str,
        help = "[optional] output directory - if unspecified, input folder will be used"
    )
    parser.add_argument(
        "--label_every", default = 1, required = False, type = int,
        help = "[optional] label only every N-th cloud (starting after the 'precede_with')"
    )
    parser.add_argument(
        "--precede_with", default = 10, required = False, type = int,
        help = "[optional] assign (and copy) the previous N frames as 'unlabeled_pc'"
    )
    parser.add_argument(
        "--ratio", default = [1.0, 1.0, 1.0], required = False, nargs = 3, type = float,
        help = "[optional] ratio of 'train', 'test' and 'validate' data to sort the output into"
    )
    parser.add_argument(
        "--frames_from", default = 0, required = False, type = int,
        help = "[optional] ignore first N frames"
    )
    parser.add_argument(
        "--frames_max", default = -1, required = False, type = int,
        help = "[optional] stop after N frames"
    )
    parser.add_argument(
        "--tf_parent", default = "map", required = False, type = str,
        help = "[optional] parent frame to look for in TF messages (default: map)"
    )
    parser.add_argument(
        "--tf_child", default = "base_link", required = False, type = str,
        help = "[optional] child frame to look for in TF messages (default: base_link)"
    )
    args = parser.parse_args()
    if (args.dir_out == ""):
        dir_out = os.path.dirname(args.file_in)
    else:
        dir_out = args.dir_out

    precede_with = abs(args.precede_with)
    label_every = abs(args.label_every)

    ratio = [1,1,1]
    if (len(args.ratio) == 3):
        s = 0
        for i in range(3):
            ratio[i] = abs(float(args.ratio[i]))
            s += ratio[i]
        ratio = list(map(lambda x: x/s, ratio))
    elif (len(args.ratio)):
        print("ERROR!")

    *fields_param, = args.fields
    for i in range(len(fields_param)):
        if not fields_param[i] in pcl_fields:
            print(f"ERROR! Missing field '{fields_param[i]}'! AAAAAAAAAA!")
            exit()
        fields_param[i] = pcl_fields[fields_param[i]]

    manage_dirs(dir_out)

    if not os.path.isfile(f"{dir_out}/metadata.json"):
        print("Metadata file does not exist, generating data from mcap...")
        meta_dict["info"]["source"] = os.path.basename(args.file_in)
        meta_dict["info"]["user"] = gt.getuser()
        print("Processing mcap...")

        reader, pcl_count, odom_count, msg_count = init_reader(args.file_in, args.pcl_in, args.odom_in)
        rlv_count = pcl_count + odom_count
        m_l, p_l, o_l, r_l = len(str(msg_count)), len(str(pcl_count)), len(str(odom_count)), len(str(rlv_count))
        bin_files = []
        pcl_timestamps = []
        odom_data = []
        cnt = 0
        irlv = 0
        msg0 = {}
        pcls = 0
        frames_until = pcl_count
        if args.frames_max != -1:
            frames_until = args.frames_max * args.label_every + args.frames_from + args.precede_with

        # We'll gather TFs for map->base_link here
        tf_list = []  # list of {'timestamp': float, 'H': 4x4 np.array}
        tf_topics = ["/tf", "/tf_static", "tf", "tf_static"]  # some bags use no leading slash

        # read messages (include pointcloud, odom, and tf topics)
        topics_to_read = [args.pcl_in, args.odom_in] + tf_topics
        for topic, msg, timestamp in read_messages(reader, topics_to_read):
            # ----------------------------------------------------------------
            # Collect TF messages first (offline). TF message type: tf2_msgs/TFMessage (field 'transforms')
            if topic in tf_topics:
                # msg.transforms is a list of TransformStamped
                try:
                    for t in msg.transforms:
                        # header.frame_id is parent, child_frame_id is child
                        parent = t.header.frame_id
                        child = t.child_frame_id
                        if parent == args.tf_parent and child == args.tf_child:
                            # timestamp may be in t.header.stamp
                            try:
                                ts = t.header.stamp.sec+ t.header.stamp.nanosec * 1e-9 
                            except Exception:
                                ts = 0.0
                            H = transform_to_matrix(t.transform)
                            tf_list.append({'timestamp': ts, 'H': H})
                except Exception:
                    # older or different TF message structure: try alternative access
                    pass
                # continue reading; TF messages themselves are not written as point clouds
            # ----------------------------------------------------------------
            elif isinstance(msg, PointCloud2):
                if (args.frames_from <= pcls < frames_until):
                    # get closest TF for this pointcloud's timestamp
                    pc_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 
                    # sort tf list if not sorted yet (do it lazily once list is non-empty)
                    if tf_list:
                        # ensure sorted by timestamp
                        tf_list = sorted(tf_list, key=lambda x: x['timestamp'])
                        H_dynamic = get_closest_tf_matrix(pc_time, tf_list)
                    else:
                        H_dynamic = np.eye(4, dtype=np.float64)  # fallback

                    # read pointcloud fields to numpy
                    pointcloud_np_structured = point_cloud2.read_points(msg, field_names=tuple(fields_param), skip_nans=True)
                    pnps = []
                    for f in fields_param:
                        pnps.append(pointcloud_np_structured[f])
                    pointcloud_np = np.stack(pnps, axis=-1)

                    # sensor->base transform (your existing fixed transform)
                    H_sensor_base = np.eye(4, dtype=np.float32)
                    H_sensor_base[:3, :3] = rot.from_euler("xyz", [0.0, 0.0, np.pi], degrees = True).as_matrix()
                    H_sensor_base[:3, 3] = [0.466, 0.0, 0.849]

                    # # compose: map <- base <- sensor  => map <- sensor: H_map_sensor = H_dynamic @ H_sensor_base
                    # H_combined = (H_dynamic.astype(np.float32) @ H_sensor_base.astype(np.float32)).astype(np.float32)

                    # transform pcl from sensor coord sys to map coord sys
                    tf_pointcloud_np = transform_points(pointcloud_np, H_sensor_base, len(fields_param)).astype(np.float32)

                    fname = f"{len(pcl_timestamps):07d}"
                    bin_files.append(fname)
                    pcl_timestamps.append(pc_time)
                    fname = f"{dir_out}/points/{fname}"
                    with open(fname, 'wb') as f:
                        b = tf_pointcloud_np.tobytes()
                        f.write(b)
                pcls += 1

        #     elif isinstance(msg, Odometry) or isinstance(msg, PoseStamped):
        #         # Get the odometry message timestamp
        #         odom_time = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
    
        #         # Get the closest map-to-base_link transform for this odometry message
        #         if tf_list:
        #             tf_list = sorted(tf_list, key=lambda x: x['timestamp'])
        #             H_dynamic = get_closest_tf_matrix(odom_time, tf_list)
        #         else:
        #             H_dynamic = np.eye(4, dtype=np.float64)
    
        #         # Extract the original odometry data
        #         odom = extract_odom(msg, isinstance(msg, PoseStamped), msg0)
    
        #         # Transform the odometry pose to map frame
        #         if isinstance(msg, Odometry):
        #             # For Odometry messages, transform both pose and twist if needed
        #             pose_matrix = np.eye(4)
        #             pose_matrix[:3, 3] = [odom['x'], odom['y'], odom['z']]
        #             pose_matrix[:3, :3] = rot.from_euler('z', odom['yaw']).as_matrix()
        
        #             # Apply map transformation
        #             transformed_pose = H_dynamic @ pose_matrix
        
        #             # Update odometry data
        #             odom['x'], odom['y'], odom['z'] = transformed_pose[:3, 3]
        #             odom['yaw'] = rot.from_matrix(transformed_pose[:3, :3]).as_euler('xyz')[2]
        
        # # Note: Velocity/twist data would need to be transformed too if needed
        # # For simplicity, we'll leave it in the body frame here
        
        #         elif isinstance(msg, PoseStamped):
        # # For PoseStamped, just transform the pose
        #             pose_matrix = np.eye(4)
        #             pose_matrix[:3, 3] = [odom['x'], odom['y'], odom['z']]
        #             pose_matrix[:3, :3] = rot.from_euler('z', odom['yaw']).as_matrix()
        
        # # Apply map transformation
        #             transformed_pose = H_dynamic @ pose_matrix
        
        # # Update odometry data
        #             odom['x'], odom['y'], odom['z'] = transformed_pose[:3, 3]
        #             odom['yaw'] = rot.from_matrix(transformed_pose[:3, :3]).as_euler('xyz')[2]
    
        #         odom_data.append(odom)


            # elif isinstance(msg, (Odometry, PoseStamped)):
            #     # Get current map->base_link transform
            #     odom_time = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
            #     H_map_base = get_closest_tf_matrix(odom_time, tf_list) if tf_list else np.eye(4)
                
            #     # Convert map-frame odom to base_link frame
            #     if isinstance(msg, Odometry):
            #         # Pose in map frame
            #         pose_map = np.eye(4)
            #         pose_map[:3, 3] = [msg.pose.pose.position.x, 
            #                         msg.pose.pose.position.y, 
            #                         msg.pose.pose.position.z]
            #         pose_map[:3, :3] = rot.from_quat([
            #             msg.pose.pose.orientation.x,
            #             msg.pose.pose.orientation.y,
            #             msg.pose.pose.orientation.z,
            #             msg.pose.pose.orientation.w
            #         ]).as_matrix()
                    
            #         # Convert to base_link frame: T_base_map = T_base_map = inv(T_map_base)
            #         pose_base = np.linalg.inv(H_map_base) @ pose_map
                    
            #         # Extract components
            #         odom = {
            #             'timestamp': odom_time,
            #             'x': pose_base[0, 3],
            #             'y': pose_base[1, 3],
            #             'z': pose_base[2, 3],
            #             'yaw': rot.from_matrix(pose_base[:3, :3]).as_euler('zyx')[0],
            #             'vx': msg.twist.twist.linear.x,
            #             'vy': msg.twist.twist.linear.y,
            #             'yawrate': msg.twist.twist.angular.z
            #         }

            #         odom_data.append(odom)

            elif isinstance(msg, PoseStamped):
                # Get current map->base_link transform
                odom_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                H_map_base = get_closest_tf_matrix(odom_time, tf_list) if tf_list else np.eye(4)
                    
                # Pose in map frame
                pose_map = np.eye(4)
                pose_map[:3, 3] = [msg.pose.position.x, 
                                    msg.pose.position.y, 
                                    msg.pose.position.z]
                pose_map[:3, :3] = rot.from_quat([
                        msg.pose.orientation.x,
                        msg.pose.orientation.y,
                        msg.pose.orientation.z,
                        msg.pose.orientation.w
                    ]).as_matrix()
                    
                    # Convert to base_link frame: T_base_map = inv(T_map_base)
                pose_base = np.linalg.inv(H_map_base) @ pose_map
                    
                    # Extract components (PoseStamped has no velocity data)
                odom = {
                        'timestamp': odom_time,
                        'x': pose_base[0, 3],
                        'y': pose_base[1, 3],
                        'z': pose_base[2, 3],
                        'yaw': rot.from_matrix(pose_base[:3, :3]).as_euler('zyx')[0],
                        'vx': 0.0,  # PoseStamped doesn't contain velocity
                        'vy': 0.0,
                        'yawrate': 0.0
                    }

                odom_data.append(odom)


            # elif isinstance(msg, (Odometry, PoseStamped)):
            #     odom_time = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
                
            #     # Get map->base_link transform
            #     H_map_base = get_closest_tf_matrix(odom_time, tf_list) if tf_list else np.eye(4)
                
            #     # Extract original odom (in map frame)
            #     odom_map_frame = extract_odom(msg, isinstance(msg, PoseStamped))
                
            #     # Convert to base_link frame
            #     pose_map = np.eye(4)
            #     pose_map[:3, 3] = [odom_map_frame['x'], odom_map_frame['y'], odom_map_frame['z']]
            #     pose_map[:3, :3] = rot.from_euler('z', odom_map_frame['yaw']).as_matrix()
                
            #     pose_base = np.linalg.inv(H_map_base) @ pose_map
                
            #     # Create final odom entry
            #     odom = {
            #         'timestamp': odom_time,
            #         'x': pose_base[0, 3],
            #         'y': pose_base[1, 3],
            #         'z': pose_base[2, 3],
            #         'yaw': rot.from_matrix(pose_base[:3, :3]).as_euler('zyx')[0],
            #         'vx': odom_map_frame.get('vx', 0.0),
            #         'vy': odom_map_frame.get('vy', 0.0),
            #         'yawrate': odom_map_frame.get('yawrate', 0.0)
            #     }
            #     odom_data.append(odom)

            # use the functions below without msg0 for absolute values
            elif isinstance(msg, Odometry):
                odom_data.append(extract_odom(msg, False, msg0))
            elif isinstance(msg, PoseStamped):
                odom_data.append(extract_odom(msg, True, msg0))
            else:
                irlv += 1  # irrelevant

            cnt += 1
            sys.stdout.write(f"\rMessages processed - total: [{cnt :{m_l}d} / {msg_count :{m_l}d}]"
                             f" (relevant: [{cnt - irlv :{r_l}d} / {rlv_count :{r_l}d}] -"
                             f" pcl: [{pcls :{p_l}d} / {pcl_count :{p_l}d}],"
                             f" odom: [{len(odom_data) :{o_l}d} / {odom_count :{o_l}d}] )")
            sys.stdout.flush()
        print(f"\n...mcap processing done.")
        ptl = len(pcl_timestamps)
        if (ptl < pcl_count): print(f" ({pcl_count - ptl} pcl frames skipped)")

        # ... the rest of your pipeline is unchanged: timestamps -> odom association, metadata, renaming, etc.
        print("Assigning timestamps (odom -> pcl) and file hashes...")
        stamps = []
        hashes = []
        cnt = 1
        f_l = len(str(len(bin_files)))
        # normalize pcl_timestamps to start from zero (as your code did)
        pcl_timestamps = (np.array(pcl_timestamps) - pcl_timestamps[0]).tolist()
        for i in bin_files:
            id = int(os.path.basename(i))
            stamps.append(odom_data[get_closest_stamp_id(pcl_timestamps[id], odom_data)])
            with open(f"{dir_out}/points/{i}", "rb") as f:
                b = f.read()
                hashes.append(hashlib.md5(b).hexdigest())
            sys.stdout.write(f"\rFiles processed: {cnt :{f_l}d} / {len(bin_files) :{f_l}d}")
            sys.stdout.flush()
            cnt += 1

        # for i in bin_files:
        #     id = int(os.path.basename(i))
        #     pcl_time = pcl_timestamps[id]  # Ensure this is in seconds (same units as odom_data)
    
        #     # Get synchronized odometry (with interpolation)
        #     odom = get_interpolated_odom(
        #         stamp=pcl_time,
        #         odom_data=odom_data,
        #         max_diff_threshold_ns=0.1,  # 100ms tolerance
        #         interpolate=True         # Enable for smoother motion
        #     )
    
        #     if odom is None:
        #         print(f"Warning: No odometry within 100ms for point cloud {i} (t={pcl_time:.3f}s)")
        #         continue
    
        #     stamps.append(odom)  # Store synchronized odometry
    
        #     # Process point cloud file
        #     with open(f"{dir_out}/points/{i}", "rb") as f:
        #         b = f.read()
        #         hashes.append(hashlib.md5(b).hexdigest())
    
        #     # Progress reporting
        #     cnt += 1
        #     sys.stdout.write(f"\rFiles processed: {cnt:{f_l}d}/{len(bin_files):{f_l}d}")
        #     sys.stdout.flush()

        print("\nOdometry-point cloud synchronization complete.")
        print(f"Success rate: {len(stamps)}/{len(bin_files)} ({100*len(stamps)/len(bin_files):.1f}%)")

        print("\n...data assigned.")
        print("Generating layout links...")
        label_files = []
        link_fwd = {}
        link_bwd = {}
        bid = 1
        for i in range(precede_with, len(bin_files), label_every):
            for j in range(precede_with):
                current_file = f"{dir_out}/points/{i - precede_with + j :07d}"
                result_file = f"{dir_out}/unlabeled_pc/{bid :07d}_{j+1}.bin"
                if current_file in link_fwd:
                    link_fwd[current_file].append(result_file)
                else: link_fwd[current_file] = [result_file]
                link_bwd[result_file] = current_file
            bid += 1
            label_files.append(f"{dir_out}/points/{i :07d}")
        print("Generating metadata...")
        for id in range(1, len(label_files)+1):
            oid = (id - 1) * label_every + precede_with
            meta_dict["data"].append({
                "id": id,
                "odom": stamps[oid],
                "pointcloud": {
                    "file": f"points/{id :07d}.bin",
                    "checksum": hashes[oid],
                },
                "labels": {
                    "file": f"labels/{id :07d}.txt",
                    "checksum": "",
                },
                "unlabeled_clouds": []
            })
            for i in range(precede_with):
                ucid = oid - precede_with + i
                meta_dict["data"][-1]["unlabeled_clouds"].append(
                    {
                        "file": f"unlabeled_pc/{id :07d}_{i+1}.bin",
                        "checksum": hashes[ucid],
                        "odom": stamps[ucid]
                    }
                )
        print("...metadata generated, saving...")
        save_meta(meta_dict, dir_out)
        print(f"... 'metadata.json' saved to: '{dir_out}'.")

        print("Rearranging files...")
        print("...duplicating 'unlabeled'...")
        tot, ctot, c = 0, 1, 1
        kl = len(link_fwd.keys())
        for i in link_fwd.keys():
            tot += len(link_fwd[i])
        c_l, tot_l = len(str(kl)), len(str(tot))

        for i in link_fwd.keys():
            l = len(link_fwd[i])
            l_l = len(str(l))
            p = 1
            for j in link_fwd[i]:
                shutil.copy2(i, j)
                sys.stdout.write(f"\r - files copied: {ctot :{tot_l}d} / {tot :{tot_l}d}"
                                 f"\t[ file #{c :{c_l}d} / {kl :{c_l}d} "
                                 f"-> duplicate #{p :{l_l}d} / {l :{l_l}d} ]")
                sys.stdout.flush()
                p += 1
                ctot += 1
            c += 1

        print("\n...renaming files...")
        id = 1
        tot = len(label_files)
        tot_l = len(str(tot))
        for i in label_files:
            os.rename(f"{i}", f"{dir_out}/points/{id :07d}.bin")
            sys.stdout.write(f"\r - files renamed: {id :{tot_l}d} / {tot :{tot_l}d}")
            sys.stdout.flush()
            id += 1

        print("\n...removing excess files...")
        cnt = 0
        tot = len(bin_files) - len(label_files)
        l = len(str(tot))
        for i in bin_files:
            i = f"{dir_out}/points/{i}"
            if os.path.isfile(i):
                os.remove(i)
                cnt += 1
            sys.stdout.write(f"\r - files removed: {cnt :{l}d} / {tot :{l}d}")
            sys.stdout.flush()
        print("\n...excess files removed.")

        print("Done.")
        print("(Run this script again once you have all the labels"
              " in the '/labels/' folder of your output directory!)")
    else:
        print("Metadata file found, updating label checksums...")
        src_files = os.listdir(f"{dir_out}/points/")
        txt_files = os.listdir(f"{dir_out}/labels/")
        if (len(src_files)==len(txt_files)):
            lbl_hashes = {}
            for i in (txt_files):
                id = i[:-4]
                if not os.path.isfile(f"{dir_out}/labels/{id}.txt"):
                    print(f"Missing: 'labels/{id}.txt'.")
                else:
                    with open(f"{dir_out}/labels/{i}", "rb") as f:
                        b = f.read()
                        lbl_hashes[id] = hashlib.md5(b).hexdigest()

            with open(f"{dir_out}/metadata.json", "r") as f:
                meta_in = json.load(f)
                for k, v in lbl_hashes.items():
                    meta_in["data"][int(k)-1]["labels"]["checksum"] = v
            print("Rewriting data...")
            with open(f"{dir_out}/metadata.json", "w") as f:
                json.dump(meta_in, f, indent = 4)
            print("...data saved.")

            print("Writing IDs to the corresponding files by the given dataset ratio...")
            for i in range(len(datasets)):
                with open(f"{dir_out}/ImageSets/{datasets[i]}", "w") as f:
                    for j in np.arange(i, len(txt_files), 1/ratio[i]):
                        f.write( txt_files[int(j)][:-4] + '\n')
                    print(f"- file '{dir_out}/ImageSets/{datasets[i]}' created.")
            print("...files created, IDs allocated.")

            print("All done.")
            print("The data is ready for use.")
        else:
            print("The files in the /labels/ folder do not match their binary counterparts!"
                  " (make sure that for every '.bin' file in the '/points/' folder"
                  "there is a '.txt' file [containing the labels] in the '/labels/' folder"
                  "and that there are no other files present!)")

if __name__ == "__main__":
    main()
