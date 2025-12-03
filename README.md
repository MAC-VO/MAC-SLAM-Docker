# MAC-SLAM-Docker
| Docker container

- `L4T-AGX`: legacy docker for AGX Orin.
- `L4T-Thor`: legacy docker
- `L4T-AGX-Orin`: latest docker for TensorRT on AGX Orin.
- `Linux`: docker for normal NVIIDA GPU devvices (ex. 3090Ti).
    - services
        - `dev`: Legacy sevice
        - `trt-dev`: Current service. It also can run Pytorch MAC-VO and quantized MAC-VO.
    - base iamges
        1. `nvcr.io/nvidia/tensorrt:25.03-py3`
        2. `yutianchen/macslam:aug2025-v2`
