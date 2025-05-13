FROM dustynv/l4t-pytorch:2.2-r35.4.1

# Install additional development tools (if needed)
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Set up the working directory
WORKDIR /home/root/workspace

# Install HDF5 dependency for SLAM keyframe storage and data persistent storage.
RUN apt-get update && apt-get install -y \
    libhdf5-dev \
    libhdf5-serial-dev \
    hdf5-tools \
    && rm -rf /var/lib/apt/lists/*

COPY ./requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt && rm /tmp/requirements.txt
RUN git config --global --add safe.directory '*'

USER root

# Expose rerun visualization ports
EXPOSE 9876 9877 9090

CMD ["bash"]
