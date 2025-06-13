#todo: merge with mcap extractor
import argparse
import json
import sys
import os

def rot_from_quat(x,y,z,w): #todo: you know... THE WHOLE THING
    return 0.0

print("Hi! This tool converts an annotation json file "
      "with the default format of 'Segments.ai' and converts it "
      "into several txt files suitable to use in MMdetection3D.")
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("file_in", help = "input file (json) to convert")
parser.add_argument("--dir_out", required = False, help = "output folder")
args = parser.parse_args()
file_in, dir_out = args.file_in, args.dir_out
print("Opening file...")
with open(file_in) as f:
    data_in = json.load(f)
print("...file processed...")
cats = {}
for i in data_in['dataset']['task_attributes']['categories']:
    cats[i['id']] = i['name']
print("...data loaded.")

print("Generating output...")
if not os.path.exists(dir_out):
    os.makedirs(dir_out)
    print(f"Directory '{dir_out}' created.")
annot = {}
jt = len(data_in['dataset']['samples'])
jl = len(str(jt))
jc = 0
for j in data_in['dataset']['samples']:
    jc += 1
    fn_out = f"{dir_out}/{j['name'][:-4]}.txt"
    # todo: if exists: scream or sth!
    with open(fn_out, "w") as f:
        it = len(j["labels"]["ground-truth"]["attributes"]["annotations"])
        il = len(str(it))
        ic = 0
        for i in j["labels"]["ground-truth"]["attributes"]["annotations"]:
            ic += 1
            f.write(" ".join([
                    str(i['position']['x']),
                    str(i['position']['y']),
                    str(i['position']['z']),
                    str(i['dimensions']['x']),
                    str(i['dimensions']['y']),
                    str(i['dimensions']['z']),
                    str(rot_from_quat(
                        i['rotation']['qx'],
                        i['rotation']['qy'],
                        i['rotation']['qz'],
                        i['rotation']['qw'])),
                    #i['yaw']', #the same? - todo: check
                    cats[i['category_id']],
                    ]) + "\n" )
            sys.stdout.write(f"\rProgress: [File {jc:{jl}d} / {jt:{jl}d}]: Label {ic:{il}d} / {it:{il}d}")
            sys.stdout.flush()
print("\n")
print(f"... {jc} files created at '{dir_out}'.")
print("\nAll done.")