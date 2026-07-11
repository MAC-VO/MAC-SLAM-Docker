#!/bin/bash
set -e

ARCH=$(uname -m)
DETAIL_ARCH=$(uname -r)

# cuDSS backs two consumers:
#   - C level:  Src/CUExt/SparseBA (sparse_solve extension links libcudss)
#   - Python:   nvmath-python DirectSolver (LoopPGO fused GPU direct solver)
# nvmath-python 1.0.0 hard-requires cuDSS 0.8.x, so keep these two in lockstep.
CUDSS_VERSION="0.8.0"
NVMATH_VERSION="1.0.0"

install_cudss_deb() {
    # $1: local-repo deb name. The deb ships both cudss-cuda-12 and cudss-cuda-13
    # variants; update-alternatives selects the one matching the image CUDA.
    wget "https://developer.download.nvidia.com/compute/cudss/${CUDSS_VERSION}/local_installers/$1" && \
        dpkg -i "$1" && \
        cp /var/"${1%%_*}"/cudss-*-keyring.gpg /usr/share/keyrings/ && \
        apt-get update && \
        apt-get -y install cudss && \
        rm "./$1"
}

install_nvmath_python() {
    # nvmath-python 1.0.0 requires numpy>=1.25. Older NGC bases (e.g. 24.09,
    # cu126 image) freeze numpy 1.24, which conflicts. We let numpy float up
    # WITHIN the 1.x series (>=1.25,<2) only where needed.
    MERGED_CONSTRAINTS="/tmp/constraints.nvmath.txt"
    { cat /tmp/constraints.txt 2>/dev/null; echo; \
      [ -n "${PIP_CONSTRAINT:-}" ] && [ -f "${PIP_CONSTRAINT:-}" ] && cat "$PIP_CONSTRAINT"; } \
      | grep -viE '^numpy[[:space:]]*[=<>!~]' > "$MERGED_CONSTRAINTS" || true

    BINDINGS_SPEC=""
    if [[ "$CUDA_MAJOR_VERSION" = "13" ]]; then
        BINDINGS_SPEC="cuda-bindings>=13.0.1,<13.1"
    elif [[ "$CUDA_MAJOR_VERSION" != "12" ]]; then
        echo "!!! Error: Unsupported CUDA Major Version: $CUDA_MAJOR_VERSION"
        exit 1
    fi
    # PIP_CONSTRAINT override points pip at the merged (numpy-stripped) file so the
    # default NGC env constraint can't re-pin numpy behind our back.
    PIP_CONSTRAINT="$MERGED_CONSTRAINTS" \
    pip install --no-cache-dir --upgrade-strategy only-if-needed \
        "numpy>=1.25,<2" "nvmath-python==${NVMATH_VERSION}" ${BINDINGS_SPEC:+"$BINDINGS_SPEC"}
}

if [[ "$ARCH" = "x86_64" ]]; then

    # x86-64: CUDA 12 and CUDA 13 images share the same deb (see install_cudss_deb).
    install_cudss_deb "cudss-local-repo-ubuntu2404-${CUDSS_VERSION}_${CUDSS_VERSION}-1_amd64.deb"
    install_nvmath_python

elif [[ "$DETAIL_ARCH" =~ "tegra" && "$ARCH" = "aarch64" ]]; then

    # Jetpack (NVIDIA Orin)
    install_cudss_deb "cudss-local-tegra-repo-ubuntu2404-${CUDSS_VERSION}_${CUDSS_VERSION}-1_arm64.deb"
    install_nvmath_python

elif [[ "$ARCH" = "aarch64" ]]; then

    # ARM SBSA (NVIDIA Thor)
    install_cudss_deb "cudss-local-repo-ubuntu2404-${CUDSS_VERSION}_${CUDSS_VERSION}-1_arm64.deb"
    install_nvmath_python

else
    echo "Error: Unsupported architecture: $ARCH"
    exit 1
fi
