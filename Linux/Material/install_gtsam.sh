# Limit build parallelism for concurrent docker compose builds (override with MAX_JOBS env if needed)
_DEFAULT_JOBS=$(( $(nproc) / 4 ))
_DEFAULT_JOBS=$(( _DEFAULT_JOBS < 2 ? 2 : _DEFAULT_JOBS ))
export MAX_JOBS="${MAX_JOBS:-$_DEFAULT_JOBS}"


cd /tmp && \
  wget https://github.com/borglab/gtsam/archive/refs/tags/4.3a1.tar.gz && \
  tar zxvf 4.3a1.tar.gz && \
  cd gtsam-4.3a1 && \
  pip install -r python/requirements.txt && \
  mkdir build && \
  cd build && \
  cmake .. -DGTSAM_BUILD_PYTHON=1 -DGTSAM_PYTHON_VERSION=3.10.12 -DGTSAM_BUILD_UNSTABLE:OPTION=ON -DCMAKE_BUILD_TYPE=Release -DGTSAM_WITH_TBB=OFF && \
  make -j"${MAX_JOBS}" && \
  make python-install

cd /tmp/gtsam-4.3a1 && \
  rm -rf build && \
  mkdir build && \
  cd build && \
  cmake .. -DGTSAM_BUILD_UNSTABLE:OPTION=ON -DCMAKE_BUILD_TYPE=Release -DGTSAM_CMAKE_CXX_FLAGS="-march=native" -DGTSAM_WITH_TBB=OFF && \
  sudo make install && \
  rm -rf /tmp/gtsam-4.3a1
