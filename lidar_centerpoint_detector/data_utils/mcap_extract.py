from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
import numpy as np
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

from scipy.spatial.transform import Rotation as rot
import os
import sys
import datetime as dt
import getpass as gt
import argparse
import json
import hashlib

reqdirs = ["unlabeled_pc", "points", "labels", "ImageSets"]
datasets = ["train.txt", "val.txt", "test.txt"]
label_every = 10
ratio = [1, 1, 1]

meta_dict = {
    "info": {
        "tool": "mcap_extract.py",
        "version": "0.2",
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

def get_closest_stamp_id(stamp: int, stamps: list): # todo: optimize
    if not len(stamps): return -1
    cid = 0
    closest = stamp - int(stamps[0]["timestamp"])
    for i in range(1, len(stamps)):
        current = abs(stamp - stamps[cid]["timestamp"])
        if current < closest:
            closest = current
            cid = i
    return cid

def save_meta(meta_dict, dir_out):
    print("Generating metadata file...")
    with open(f"{dir_out}/metadata.json", "w") as f:
        meta_dict["info"]["last_updated"] = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        json.dump(meta_dict, f, indent=4)
    print("...metadata file created.") #todo: check file

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

def read_messages(reader):
    topic_types = reader.get_all_topics_and_types()
    def typename(topic_name):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type = get_message(typename(topic))
        msg = deserialize_message(data, msg_type)
        yield topic, msg, timestamp
    del reader

def main():
    print("Welcome to the annotation preprocessor! (MCAP to bin + folder structure + metadata.json)")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "file_in", help = "input bag path (folder or filepath) to read from"
    )
    parser.add_argument(
        "pcl_in", help = "name of the pointcloud to use"
    )
    parser.add_argument(
        "odom_in", help = "name of the odom topic to use"
    )
    parser.add_argument(
        "--dir_out", default = "", required = False,
        help = "[optional] output directory - if unspecified, input folder will be used"
    )
    parser.add_argument(
        "--ratio", default = "", required = False, nargs = 3,
        help = "[optional] ratio of 'train', 'test' and 'validate' data to sort the output into"
    )
    
    args = parser.parse_args()
    if (args.dir_out == ""):
        dir_out = os.path.dirname(args.file_in) # todo: check existance
    else: dir_out = args.dir_out # todo: handle directory-as-input

    ratio = [1,1,1]
    if (len(args.ratio) == 3):
        print(args.ratio)
        s = 0
        for i in range(3):
            ratio[i] = float(args.ratio[i]) # conversion
            s += ratio[i]
        ratio = list(map(lambda x: x/s, ratio))
    elif (len(args.ratio)): # (if 0 < input != 3)
        print("ERROR!") # todo: specify (+ suggest?)

    manage_dirs(dir_out)
    
    if not os.path.isfile(f"{dir_out}/metadata.json"):
        print("Metadata file does not exist, generating data from mcap...")
        meta_dict["info"]["source"] = os.path.basename(args.file_in) # should use ntpath ! (or check if so...)
        meta_dict["info"]["user"] = gt.getuser()
        print("Processing mcap...")
        reader, pcl_count, odom_count, msg_count = init_reader(args.file_in, args.pcl_in, args.odom_in)
        pcl_timestamps = {}
        odom_data = []
        cnt = 1
        for topic, msg, timestamp in read_messages(reader):
            if isinstance(msg, PointCloud2):
                pointcloud_np_structured = point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
                pointcloud_np = np.stack([pointcloud_np_structured['x'],
                            pointcloud_np_structured['y'],
                            pointcloud_np_structured['z'],
                            pointcloud_np_structured['intensity']], axis=-1)
                if (len(pcl_timestamps) % label_every): # has remainder (unlabeled)
                    suffix = f"_{(len(pcl_timestamps) % label_every)}" # suffix = remainder
                else: suffix = ""
                fname = f"{((len(pcl_timestamps) // label_every) +1 ):07d}{suffix}"
                pcl_timestamps[fname] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                if len(suffix): fname = f"{dir_out}/unlabeled_pc/{fname}.bin" # add path and suffix (for unlabeled)
                else: fname = f"{dir_out}/points/{fname}.bin" # add path and suffix (for labeled)
                with open(fname, 'wb') as f:
                    b = pointcloud_np.tobytes() # todo: compare with inline performance
                    f.write(b)
            elif isinstance(msg, Odometry):
                o = msg.pose.pose.orientation
                x, y, z, w = o.x, o.y, o.z, o.w
                yaw = rot.from_quat([x, y, z, w]).as_euler("xyz")[2]
                odom_data.append({
                    "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                    "x": msg.pose.pose.position.x,
                    "y": msg.pose.pose.position.y,
                    "z": msg.pose.pose.position.z,
                    "yaw": yaw,
                    "vx": msg.twist.twist.linear.x,
                    "vy": msg.twist.twist.linear.y,
                    "yawrate": msg.twist.twist.angular.z
                })
            sys.stdout.write(f"\rMessages processed:\t- total: {cnt :7d} / {msg_count :7d}\t(pcl: {len(pcl_timestamps) :7d} / {pcl_count :7d}, odom: {len(odom_data) :7d} / {odom_count :7d})")
            sys.stdout.flush()
            cnt += 1
        print("\n...mcap processing done.")

        print("Assigning timestamps... (odom -> pcl)")
        bin_files = os.listdir(f"{dir_out}/points/") + os.listdir(f"{dir_out}/unlabeled_pc/")
        filehash = ""
        cnt = 1
        for i in bin_files:
            id = os.path.basename(i).split('.')[0] # filename that contains id
            
            # todo: check filename format
            if (len(id) == 2 and id[1]==".bin") or True:
                cf = id[0].split('_')
                if (cf[0].isdecimal() and cf[-1].isdecimal()) or True: # confirmed
                    if '_' in i: fdir = f"{dir_out}/unlabeled_pc" # add path for unlabeled
                    else: fdir = f"{dir_out}/points" # add path for [to be] labeled
                    with open(f"{fdir}/{i}", "rb") as f: # ...extract hash
                        b = f.read()
                        filehash = hashlib.md5(b).hexdigest()
                else: continue # denied: skip file
            else: continue # denied: skip file
            
            if ('_' in id):
                meta_dict["data"][int(id.split('_')[0])-1]["unlabeled_clouds"].append(
                    {
                        "file": f"unlabeled_pc/{id}.bin",
                        "checksum": filehash,
                        "odom": odom_data[get_closest_stamp_id(pcl_timestamps[id], odom_data)]
                    }
                )
            else:
                meta_dict["data"].append({
                        "id": int(id),
                        "odom": odom_data[get_closest_stamp_id(pcl_timestamps[id], odom_data)],
                        "pointcloud": {
                            "file": f"points/{id}.bin",
                            "checksum": filehash,
                        },
                       "labels": {
                           "file": f"labels/{id}.txt",
                           "checksum": "",
                       },
                       "unlabeled_clouds": []
                    })
            sys.stdout.write(f"\rFiles processed:\t {cnt :7d} / {len(bin_files) :7d}")
            sys.stdout.flush()
            cnt += 1
        print("\n...file processing done.")
        print("Saving metadata to json file...")
        save_meta(meta_dict, dir_out)
        print("...data saved.")
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
            #todo: error handling
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
            #todo: try to resolve, check for errors, etc.

if __name__ == "__main__":
    main()