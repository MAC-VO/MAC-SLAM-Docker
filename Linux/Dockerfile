FROM pytorch/pytorch:2.7.1-cuda12.8-cudnn9-runtime

# Install additional development tools (if needed)
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Set up the working directory
WORKDIR /home/root/workspace

COPY requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt && rm /tmp/requirements.txt

# Install uniception package, use --no-deps to avoid installing dependencies that may conflict
# with the existing environment
RUN pip install uniception --no-deps 

# Configure Git
RUN git config --global --add safe.directory '*'

# Set up the working directory
WORKDIR /home/devuser/workspace

# Expose rerun visualization ports
EXPOSE 9876 9877 9090

CMD ["bash"]
