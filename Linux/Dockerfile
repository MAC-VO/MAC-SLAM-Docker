FROM pytorch/pytorch:2.4.0-cuda12.4-cudnn9-runtime

# Install additional development tools (if needed)
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    vim \
    && rm -rf /var/lib/apt/lists/*


# Create a non-root user for better security (optional but recommended)
RUN groupadd -r devgroup && useradd -m -r -g devgroup devuser

# Set up the working directory
WORKDIR /home/devuser/workspace

COPY requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt && rm /tmp/requirements.txt

USER devuser

# Expose rerun visualization ports
EXPOSE 9876 9877 9090

CMD ["bash"]
