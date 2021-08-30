## Compile
Create a build folder, and run cmake and make in that folder
```
cd build
cmake ..
make
```


## Run
Run the `detect_plane` executable in the build folder by passing it one argument, which is the path to the input .pcd file.

```
./detect_plane <path to .pcd file>
```

## Code
The task is to first detect a horizontal plane, and then to segment and cluster objects on it.
See code for comments


## Example pointcloud
Example pointclouds can be found [here](https://bib-cloud.bib.hochschule-bonn-rhein-sieg.de/apps/files/?dir=/b-it-bots-ds&fileid=3885830)
