# mcap_extract usage
## requirements
You are going to need:
- a valid ros2 distribution (properly installed and configured)
- an '.mcap' file with pointcloud and odometry data in it
- some space for the output (roughly the same as the pcl and odom data in the mcap)

## command line usage
example: "mcap_extractor.py /your_pcl_topic /your_odom_topic --out-dir /someplace/your/output_directory"
note: if no output directory is specified, the input directory will be used

## output
### What you get:

#### [1st execution]
3 subdirectories:
- labels
- points
- unlabeled_pc
(The latter 2 containing the pcl data in '.bin' format)

1 file:
- metadata.json
#### [2nd execution]
updated checksums in your metadata file

### What to do with it:
All the '.bin' files in the 'points' directory need to be labeled...
...the labels need to be in '.txt' format [todo: explain the format in detail]
After the labeling is done (using a 3rd party software) and exported into the '/labels/' directory,
the script needs to be run again [todo: different (optional) command line arguments for 2nd execution]
notes:
- currently it needs the same arguments as input for the 2nd execution, but the mcap will be completely ignored
- currently it lacks proper error handling, so:
    - the directories need to contain only the relevant files with continuous numbering
    - the '.bin' and '.txt' files (their amount and names for each) has to be the same (in '/points/' and '/labels/', respectively)
    - all the files need to be intact (rerun, delete metadata beforehand, if necessary... all else can be kept - but the generated files will be reprocessed and overwritten)
- the only difference that counts between the 2 executions is the existence of the metadata file
- everything

## TL;DR:
1. get ros2
2. get mcap
3. run py (see: command line usage)
4. recieve '*.bin's and 'metadata.json' (in output dir)
5. use something to annotate the '/points/*.bin' files
6. export labels as '*.txt' to '/labels/' folder
7. run script again (same command)
8. check results and smile! (or curse and hate me if need be)