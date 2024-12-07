# ResearchProject

To save files from the ouster scanner and a camera in a certain filestructure do the following:

1. Open a Terminal and type in
```console
sudo bash runpipe.bash
```

2. Open a new Terminal and type in
```console
cd noetic-slam/
bash scripts/docker-run.sh [none/integrated/nvidia/amd]
```

3. Run the following command either inside the docker container or in another terminal:
```console
bash scripts/combined-record.sh
```

The newly created/saved files are save to a new folder in directory noetic-slam/sampledata/raw/.


To clean up the saved data (delete data from scanner or camera if the other device did not collect data at the same time), run the following in directory ResearchProject:
```console
bash cleanupsampledata.sh <directory>
```
The used directory is the newly created directory where the data was stored in, e.g. 00000075 (in directory noetic-slam/sampledata/raw).