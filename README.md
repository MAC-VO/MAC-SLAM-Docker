# MAC-SLAM-Docker
>  Docker container for MAC-SLAM Development

## Linux PC (AMD-64) Environment

You can launch the develop environment by running `start_interact.sh`.

```bash
cd ./Docker/Linux
./start_interact.sh [PATHS_TO_MOUNT...]
```

### File System Mounting

If your `Model`, `Results` and `DATA_ROOT` directories are not in repo root directly (either in other directory or symbolic linked), you can pass additional arguments to `./start_interact.sh` to mount them in the image.

Any paths provided as argument to `start_interact.sh` will be mounted at the exact same path in image.

### CUDA Version

`start_interact.sh` will automatically select the the correct image based on your CUDA version (CUDA 12 / 13).

You can force override this choice by setting environment variable `FORCE_CUDA`.

```
$ FORCE_CUDA=12 ./start_interact.sh 
$ FORCE_CUDA=13 ./start_interact.sh
```

## Jetson Orin Linux (SBSA)

> Coming soon.
