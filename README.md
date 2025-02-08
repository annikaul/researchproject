## ResearchProject

# Save Images and Scanner data
To save files from the ouster scanner and a camera in a certain filestructure do the following:

1. Setup: Open a Terminal and type in
```console
cd chadbench
. setup_env.sh
bash setup.sh
bash scripts/.entrypoint-local.sh
```

2. Record images & lidar data
```console
bash scripts/combined-record.sh
```
The newly created/saved files are save to a new folder in directory chadbench/sampledata/raw/.


# View colored pointcloud
Samples were created from the previous steps, saved in hyperspace/hyperspace/akresearchproject/sampledata/raw. To view a sample pointcloud that is colored by a sample image, use the following command:
```console
cd chadbench
python3 -m meta_setup install -e -c projection
python3 hyperspace/akresearchproject/scripts/calibration.py
```