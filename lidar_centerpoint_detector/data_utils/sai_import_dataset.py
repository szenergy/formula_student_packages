#todo: merge with mcap extractor
import argparse
import os
import json
#from segments import SegmentsClient

def get_heading(dict: dict):

    return {
        "qx": 0,
        "qy": 0,
        "qz": 0,
        "qw": 1,
    }

print("Welcome!")
print("segments-ai python package required!\n"
      "(can be installed with:\npip install --upgrade segments-ai\n)")
print("[note: this is not a very intelligent program (as of yet), so "
      "you will keep getting this message, even if you have it installed]")

print("Waring! This program is not tested yet! It will exit for your own sake... sorry...")
input("(Press enter to exit!)")
exit()

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument(
    "dir_in", type = str, help = "input directory with pcl subdirectories, metadata, etc."
)
parser.add_argument(
    "api_key", type = str, help = "file containing (ONLY THE) api key to segments.ai"
)
parser.add_argument(
    "--raw_key", action = "store_true",
    help = "by passing this argument, you can enter you api key directly - however, it is not recomended for security reasons"
)

args = parser.parse_args()
if args.raw_key:
    api_key = args.api_key
else:
    with open(args.api_key, "r") as f:
        api_key = f.read()


dir_in = args.dir_in
files = os.listdir(f"{dir_in}/points")
files.sort
assets = []
pcl_urls = []
for file in files:
    with open(f"{dir_in}/points/{file}", 'rb') as f:
        assets.append(client.upload_asset(f, filename=file))
        pcl_urls.append(assets[-1].url)

metadata_in = f"{dir_in}/metadata.json"
data_in = {}
with open(metadata_in, 'r') as f:
    data_in = json.load(f)

frames = []
if len(files) != len(data_in['data']):
    print("Oops! Something went wrong... I fear for my dear life to continue...")
    print(f"hint: the metadata data section has {len(data_in['data'])} "
          f"and there are {len(files)} files in your 'points' dir instead.")
    input("Will you please press enter? I really want to exit now...")
    exit()
for i in data_in["data"]:
    frames.append(
        {
            "pcd": {"url": pcl_urls[i[id]], "type": "binary-xyzi"},
            "name": f"pcl_{i[id]}",
            "timestamp": i["odom"]["timestamp"],
            "ego_pose": {
                "position": {
                    "x": i["odom"]["x"],
                    "y": i["odom"]["y"],
                    "z": i["odom"]["z"]},
                "heading": get_heading(i["odom"]["yaw"])
            },
            "default_z": -1,
        },
    )

print(json.dumps(frames), indent = 4)

client = SegmentsClient(api_key)

dataset = data_in["info"]["source"] # todo: param
name = f"pcl_{dataset}_sequence" # todo: param
attributes = {"frames": frames}
sample = client.add_sample(dataset, name, attributes)
print("All done!")