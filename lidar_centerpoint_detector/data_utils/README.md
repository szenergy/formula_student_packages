# mcap_extract usage
(short summary at bottom)
## Requirements
You are going to need:
- Python
- a valid ROS2 distribution (properly installed and configured)
- an '.mcap' file with pointcloud and odometry data in it (*sensor_msgs/PointCloud2* and *nav_msgs/Odometry*, respectively)
- some space for the output (roughly the same as the pcl and odom data in the mcap)

## Command line usage
example:
    **mcap_extractor.py /your_pcl_topic /your_odom_topic --out-dir /someplace/your/output_directory --label-every 2 --precede_with 10 --ratio 2 3 2.5**  
notes:
- if no output directory is specified, the input directory will be used
- the required topic types for pcl and odom are *sensor_msgs/PointCloud2* and *nav_msgs/Odometry*, respectively
- the *label_every* [optional argument] means which *bin* files (pointclouds) to mark as to-be-labeled - starting after 'precede_with'
- the *precede_with* [optional argument] sets the number of preceding *bin* files (pointclouds) to assign to every labeled one as (preceding) unlabeled
- the *ratio* [optional argument] refers to how the data will be divided into 'train', 'validate' and 'test' datasets (respectively)
    - it is renormalized, so any valid integers and floats can be used
    - the default is setting is 1:1:1

## Output
### What you get:

#### [ 1st execution ]
- 4 subdirectories:
    - ImageSets/
    - labels/
    - points/
    - unlabeled_pc/  
(The latter 2 containing the pcl data in '.bin' format)

- 1 file:
    - metadata.json

#### [ 2nd execution ]
- *updated checksums in your metadata file*
- +3 files:
    - ImageSets/train.txt
    - ImageSets/val.txt
    - ImageSets/test.txt
(containing the IDs/filenames of the pcl, grouped/divided by the given ratio as the different datasets/categories)

### What to do with it:
All the '.bin' files in the 'points' directory need to be labeled...  
The labels need to be in '.txt' format [todo: explain the format in detail]  
After the labeling is done (using a 3rd party software) and exported into the '/labels/' directory,
the script needs to be run again [todo: different (optional) command line arguments for 2nd execution]  
notes:
- currently it needs the same arguments as input for the 2nd execution, but the mcap will be completely ignored
- currently it lacks proper error handling, so:
    - the directories need to contain only the relevant files with continuous numbering
    - the '.bin' and '.txt' files (their amount and names for each) has to be the same (in '/points/' and '/labels/', respectively)
    - all the files need to be intact (rerun, delete metadata beforehand, if necessary... all else can be kept - but the generated files will be reprocessed and overwritten)
- the only difference that counts between the 2 executions is the existence of the metadata file

## TL;DR:
1. get ros2 (https://docs.ros.org/en/jazzy/Installation.html)
2. get an mcap (with pcl and odom data [https://mcap.dev/])
3. run py (see: command line usage) [install python if you do not have it]
4. recieve '*.bin' files and 'metadata.json' (in output dir)
5. use something to annotate the '/points/*.bin' files
6. export labels as '*.txt' to '/labels/' folder [see also: sai_json_to_txt.py]
7. run script again (same command)
8. check results and smile! (or curse and hate me if need be)

# sai_json_to_txt usage
[to be integrated into mcap_extract]

## Requirements
- Python
- labels.json (input) (...)

## Command line usage
**sai_json_to_txt.py your_labels.json --dir_out /someplace/your/output_directory**

## Output
- txt files with labels - ONE PER CLOUD
    - format: "x y z l w h fi label" line for each object 

# sai_import_dataset usage - coming soon!
[to be integrated into mcap_extract]