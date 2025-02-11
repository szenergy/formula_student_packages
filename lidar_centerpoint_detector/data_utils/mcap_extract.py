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

def transform_points(pcl: np.array, H: np.array) -> np.array:
    """
    Transforms a 3D point cloud with intensity values using a homogeneous transformation matrix.

    Parameters:
        pcl (np.ndarray): An Nx4 array representing the 3D point cloud (x, y, z, intensity).
        H (np.ndarray): A 4x4 homogeneous transformation matrix.

    Returns:
        np.ndarray: The transformed 3D point cloud with intensity values preserved.
    """
    intensity = pcl[:, 3].reshape(-1,1)
    pcl = pcl[:, :3]
    pcl = np.hstack((pcl, np.ones((pcl.shape[0],1))))

    tranformed_pcl = pcl @ H.T
    tranformed_pcl = tranformed_pcl[:,:3]
    tranformed_pcl = np.hstack((tranformed_pcl, intensity))

    return tranformed_pcl

def getyaw(o):
    x, y, z, w = o.x, o.y, o.z, o.w
    return rot.from_quat([x, y, z, w]).as_euler("xyz")[2]

def extract_odom(msg, is_ps, msg0 = None):
    if is_ps:
        if msg0 is not None and len(msg0):
            return {
            "timestamp": msg.header.stamp.sec - msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "x": msg.pose.position.x - msg0["x"],
            "y": msg.pose.position.y - msg0["y"],
            "z": msg.pose.position.z - msg0["z"],
            "yaw": getyaw(msg.pose.orientation) - msg0["yaw"],
            "vx": 0.0,
            "vy": 0.0,
            "yawrate": 0.0
            }
        else: 
            odom = {
            "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "z": msg.pose.position.z,
            "yaw": getyaw(msg.pose.orientation),
            "vx": 0.0,
            "vy": 0.0,
            "yawrate": 0.0
            }
            if msg0 is not None: msg0 = odom # if first msg
            return odom
    else:
        if msg0 is not None and len(msg0):
            return {
            "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "x": msg.pose.pose.position.x - msg0["x"],
            "y": msg.pose.pose.position.y - msg0["y"],
            "z": msg.pose.pose.position.z - msg0["z"],
            "yaw": getyaw(msg.pose.pose.orientation) - msg0["yaw"],
            "vx": msg.twist.twist.linear.x - msg0["vx"],
            "vy": msg.twist.twist.linear.y - msg0["vy"],
            "yawrate": msg.twist.twist.angular.z - msg0["yawrate"]
            }
        else:
            odom = {
            "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "z": msg.pose.pose.position.z,
            "yaw": getyaw(msg.pose.pose.orientation),
            "vx": msg.twist.twist.linear.x,
            "vy": msg.twist.twist.linear.y,
            "yawrate": msg.twist.twist.angular.z
            }
            if msg0 is not None: msg0 = odom # if first msg
            return odom

def get_closest_stamp_id(stamp: int, stamps: list): # todo: optimize
    if not len(stamps): return -1
    cid = 0
    for i in range(0, len(stamps)):
        current = stamp - stamps[i]["timestamp"]
        if current <= 0.0:
            if abs(current) > abs(stamp - stamps[i-1]["timestamp"]):
                cid = i-1
            else:
                cid = i
            break
    return cid

def save_meta(meta_dict, dir_out):
    with open(f"{dir_out}/metadata.json", "w") as f:
        meta_dict["info"]["last_updated"] = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        json.dump(meta_dict, f, indent = 4)
    #print("...metadata file created.") #todo: check file

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

def read_messages(reader, pcl_topic, odom_topic):
    topic_types = reader.get_all_topics_and_types()
    def typename(topic_name):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic in [pcl_topic, odom_topic]:
            msg_type = get_message(typename(topic))
            msg = deserialize_message(data, msg_type)
            yield topic, msg, timestamp
    del reader

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
        "--dir_out", default = "", required = False, type = str,
        help = "[optional] output directory - if unspecified, input folder will be used"
    )
    parser.add_argument(
        "--label_every", default = 2, required = False, type = int,
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
    """
    parser.add_argument(
        "--frames_from", default = 0, required = False, type = int,
        help = "[optional] ignore first N frames"
    )
    parser.add_argument(
        "--frames_max", default = -1, required = False, type = int,
        help = "[optional] stop after N frames"
    )
    parser.add_argument(
        "--odom_abs", action = store_true
        help = "[optional] use the original odometry values (without subtracting the first)"
    )
    """
    
    args = parser.parse_args()
    if (args.dir_out == ""):
        dir_out = os.path.dirname(args.file_in) # TODO: check existance
    else: dir_out = args.dir_out # TODO: handle directory-as-input
    
    precede_with = abs(args.precede_with)
    label_every = abs(args.label_every)

    ratio = [1,1,1]
    if (len(args.ratio) == 3):
        s = 0
        for i in range(3):
            ratio[i] = abs(float(args.ratio[i])) # conversion
            s += ratio[i]
        ratio = list(map(lambda x: x/s, ratio))
    elif (len(args.ratio)): # (if 0 < input != 3)
        print("ERROR!") # TODO: specify (+ suggest?)

    manage_dirs(dir_out)
    
    if not os.path.isfile(f"{dir_out}/metadata.json"):
        print("Metadata file does not exist, generating data from mcap...")
        meta_dict["info"]["source"] = os.path.basename(args.file_in) # should use ntpath ! (or check if so...)
        meta_dict["info"]["user"] = gt.getuser()
        print("Processing mcap...")
        reader, pcl_count, odom_count, msg_count = init_reader(args.file_in, args.pcl_in, args.odom_in)
        m_l, p_l, o_l = len(str(msg_count)), len(str(pcl_count)), len(str(odom_count))
        bin_files = []
        pcl_timestamps = []
        odom_data = []
        cnt = 1
        msg0 = {}
        for topic, msg, timestamp in read_messages(reader, args.pcl_in, args.odom_in):
            if isinstance(msg, PointCloud2):
                #if (args.frames_max <= cnt > args.frames_from):
                pointcloud_np_structured = point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
                pointcloud_np = np.stack([pointcloud_np_structured['x'],
                            pointcloud_np_structured['y'],
                            pointcloud_np_structured['z'],
                            pointcloud_np_structured['intensity']], axis=-1)
                # TODO: make this more generic once measurement vehicle is fixed
                H = np.eye(4, dtype=np.float32)
                H[:3, :3] = rot.from_euler("xyz", [0.0, 0.0, np.pi]).as_matrix()
                H[:3, 3] = [0.466, 0.0, 0.849]
                # transform pcl from sensor coord sys to vehicle coord sys
                tf_pointcloud_np = transform_points(pointcloud_np, H).astype(np.float32)
                fname = f"{len(pcl_timestamps):07d}"
                bin_files.append(fname)
                pcl_timestamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
                fname = f"{dir_out}/points/{fname}"
                with open(fname, 'wb') as f:
                    b = tf_pointcloud_np.tobytes() # todo: compare with inline performance
                    f.write(b)

            # use the functions below without msg0 for absolute values (without subtracting the first one)
            elif isinstance(msg, Odometry): odom_data.append(extract_odom(msg, False, msg0))
            elif isinstance(msg, PoseStamped): odom_data.append(extract_odom(msg, True, msg0))

            # TODO: counter correction by skipped frames
            sys.stdout.write(f"\rMessages processed:\t- total: {cnt :{m_l}d} / {msg_count :{m_l}d}"
                             f"\t(pcl: {len(pcl_timestamps) :{p_l}d} / {pcl_count :{p_l}d},"
                             f" odom: {len(odom_data) :{o_l}d} / {odom_count :{o_l}d})")
            sys.stdout.flush()
            cnt += 1
        print(f"\n...mcap processing done.")
        ptl = len(pcl_timestamps)
        if (ptl < pcl_count): print(f"     ({ptl - pcl_count} pcl frames skipped)")

        print("Assigning timestamps (odom -> pcl) and file hashes...")
        stamps = []
        hashes = []
        cnt = 1
        f_l = len(str(len(bin_files)))
        pcl_timestamps = (np.array(pcl_timestamps) - pcl_timestamps[0]).tolist()
        for i in bin_files:
            id = int(os.path.basename(i))
            stamps.append(odom_data[get_closest_stamp_id(pcl_timestamps[id], odom_data)])
            with open(f"{dir_out}/points/{i}", "rb") as f: # ...extract hash
                b = f.read()
                hashes.append(hashlib.md5(b).hexdigest())
            sys.stdout.write(f"\rFiles processed: {cnt :{f_l}d} / {len(bin_files) :{f_l}d}")
            sys.stdout.flush()
            cnt += 1
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
            oid = (id - 1) * label_every + precede_with # original id (of the unsorted files)
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
        print("...duplicating 'unlabeled'...") #todo: if duplication is not needed  (-?)
        tot, ctot, c = 0, 1, 1 # total, and counters
        kl = len(link_fwd.keys())
        for i in link_fwd.keys():
            tot += len(link_fwd[i])
        c_l, tot_l = len(str(kl)), len(str(tot)) # format length

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
            #TODO: error handling
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
            #TODO: try to resolve, check for errors, etc.

if __name__ == "__main__":
    main()