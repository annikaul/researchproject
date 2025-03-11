### ResearchProject

## Save Images and Scanner data
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


## Stream colorized pointcloud
# Initialize
To stream a colorized pointcloud, first initialize the hyperspace project with:
```console
cd hyperspace
python3 -m meta_setup install -e -c projection
```

# Calibrate camera
For better results of a colorized pointcloud, first calibrate the camera with the commands:
```console
cd chadbench
bash scripts/calibrate-camera.sh
```

# View colored pointcloud
Finally, to stream a colorized (live) pointcloud, run the following commands:
```console
cd chadbench
bash scripts/stream-colored-pc.sh
```