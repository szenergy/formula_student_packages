#todo: merge with mcap extractor
import argparse
import os
import sys
import json
from scipy.spatial.transform import Rotation as rot

print("Welcome!")
try:
    from segments import SegmentsClient
except ImportError as ie:
    print("Uhh... hold on, something is not right!\n"
          f"[the exact error is: {ie}]\n"
          "You know that the python package 'segments-ai' is required, right?\n"
          "(you can install it with:\npip install --upgrade segments-ai")
    input("... oh, you want to exit? Just press Enter.)")
    exit()
except BaseException as be:
    print("Oopsie-doopsie! Something looks fishy!")
    print("The problem seems to be: ", be)
    input("That's all, folks! (Press Enter to exit!)")
    exit()

def dump_urls(pcl_urls):
    dumpfile = f"{os.getcwd()}/emergency_asset_url_dump.txt"
    try:
        with open(dumpfile, "a") as f:
            f.write("I am so sorry about this... Here are the raw URLs:\n")
            f.write(str(pcl_urls))
            f.write("\n---\n")
            print(f"> File {dumpfile} created.")
    except BaseException as e:
        print(f"Oh no... not the emergency dump!!! (error = {e})")

def get_heading(dict: dict):
    # TODO: use this once precise odometry is available
    # quat = rot.from_euler("xyz", [0.0, 0.0, dict]).as_quat()
    # return {
    #     "qx": quat[0],
    #     "qy": quat[1],
    #     "qz": quat[2],
    #     "qw": quat[3],
    # }
    return {
        "qx": 0.0,
        "qy": 0.0,
        "qz": 0.0,
        "qw": 1.0,
    }

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument(
    "dir_in", type = str, help = "input directory with pcl subdirectories, metadata, etc."
)
parser.add_argument(
    "api_key", type = str, help = "file containing (ONLY THE) api key to segments.ai"
)
parser.add_argument(
    "--raw_key", action = "store_true",
    help = "by passing this argument, you can enter your api key directly - however, it is not recommended for security reasons"
)
parser.add_argument(
    "--ds_name", required = False, default = '', type = str, help = "[optional] name of the dataset to be used"
)
parser.add_argument(
    "--ds_descr", required = False, default = '', type = str, help = "[optional] description of the dataset to be used"
)
parser.add_argument(
    "--sample_name", required = False, default = '', type = str, help = "[optional] name of the sample to be used"
)

args = parser.parse_args()
if args.raw_key:
    api_key = args.api_key
else:
    with open(args.api_key, "r") as f:
        api_key = f.read()

client = SegmentsClient(api_key)

dir_in = args.dir_in
files = os.listdir(f"{dir_in}/points")
files.sort()
assets = []
pcl_urls = []
l_f = len(files) # total (length) of files
sl_f = len(str(l_f)) # string length (number of output decimals)
dl_f = sl_f - 2 # progress percent decimal precision (str length)
c_f = 0 # count of uploaded files (progress)

if False: # not urls_from_file # todo: add --urls_from_file param
    print("Uploading assets...")
    for file in files:
        sys.stdout.write(f"\rFiles uploaded: {c_f:{sl_f}d} / {l_f:{sl_f}d} [{100*c_f/l_f:2.{dl_f}f}%]")
        sys.stdout.flush()
        try:
            fname = f"{dir_in}/points/{file}"
            with open(fname, 'rb') as f:
                assets.append(client.upload_asset(f, file))
                pcl_urls.append(assets[-1].url)
        except:
            dump_urls(pcl_urls)
        c_f += 1
    sys.stdout.write(f"\rAll files have been uploaded! ({c_f:{sl_f}d} / {l_f:{sl_f}d} [{100*c_f/l_f:2.{dl_f}f}%])\n")
    sys.stdout.flush()
    print("...done!")
else:
    pass # load URLs from file instead

print("Loading metadata...")
metadata_in = f"{dir_in}/metadata.json"
data_in = {}
with open(metadata_in, 'r') as f:
    data_in = json.load(f)
print("...done!")

print("Processing metadata...")
frames = []
l_d = len(data_in['data'])
sl_d = len(str(l_d))
dl_d = sl_d - 2
if len(files) != l_d:
    print("Oops! Something went wrong... I fear for my dear life to continue...")
    print(f"hint: the metadata data section has {len(data_in['data'])} relevant pcl data"
          f"but there are {len(files)} files in your 'points' dir instead.")
    dump_urls(pcl_urls)
    print("Emergency url dump executed! (might elaborate later, idk)")
    input("Will you please press enter? I really want to exit now...")
    exit()

for i in range(len(files)):
    pcl_urls.append(i)

c_d = 0
for i in data_in['data']:
    frames.append(
        {
            "pcd": {"url": pcl_urls[i['id']-1], "type": "binary-xyzi"},
            "name": f"pcl_{i['id']}",
            "timestamp": i['odom']['timestamp'],
            "ego_pose": {
                "position": {
                    "x": i['odom']['x'],
                    "y": i['odom']['y'],
                    "z": i['odom']['z']},
                "heading": get_heading(i['odom']['yaw'])
            },
        },
    )
    c_d += 1
    sys.stdout.write(f"\rpcl metadata processed: {c_d:{sl_d}d} / {l_d:{sl_d}d} [{100*c_d/l_d:2.{dl_d}f}%]")
    sys.stdout.flush()
print("...done!")

if args.ds_name:
    dataset = args.ds_name
else:
    dataset = '--'.join(data_in['info']['source'].split('.'))
if args.sample_name:
    name = args.sample_name
else:
    name = f"pcl_{dataset}_sequence"
if args.ds_descr:
    ds_descr = args.ds_descr
else:
    ds_descr = "This is a dataset of a LiDAR pointcloud sequence."
attributes = {"frames": frames}
task_type = "pointcloud-cuboid-sequence"

dsets = []
for i in client.get_datasets(user = client.get_user().username):
    dsets.append(i.name)
if not dataset in dsets:
    print("Dataset with given name not found, creating it...")
    try:
        client.add_dataset(dataset, ds_descr, task_type)
    except BaseException as e:
        print("Failed to create dataset, it says: ", e)
        dump_urls(pcl_urls)
        print("The URLs are safe tho. (or so i hope)")
        print("fun fact: pressing Enter makes me exit now.")
        exit()
    else:
        print("...dataset created successfully! (...well, without raising an exception, i mean)")

print("Adding sample...")
try:
    dataset = client.get_user().username + '/' + dataset
    sample = client.add_sample(dataset, name, attributes)
except BaseException as e:
    dump_urls(pcl_urls)
    print("Wowsies, that didn't quite work (The 'add_sample()', i mean).")
    print("That's because: ", e)
    input("The URLs had been dumped tho, so you can press Enter now. (That's my cue to exit.)")
    exit()
else:
    print("...sample added. (Yea', we happy now!)")
    
print("All done!")