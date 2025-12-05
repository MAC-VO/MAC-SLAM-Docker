# Conditional base image to port lean image for Github Action.
ARG BASE_IMAGE 

FROM ${BASE_IMAGE}

# Install additional development tools (if needed)
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    vim \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Set up the working directory
WORKDIR /home/root/workspace

COPY requirements.txt /tmp/requirements.txt
COPY additional-requirements.txt /tmp/additional-requirements.txt

RUN pip install --no-cache-dir -r /tmp/requirements.txt && rm /tmp/requirements.txt

# Install uniception package, use --no-deps to avoid installing dependencies that may conflict
# with the existing environment
RUN pip install uniception --no-deps 

# Install CUDSS (only in Dev Environment)
RUN if [ "$CI" = "false" ]; then \
        wget https://developer.download.nvidia.com/compute/cudss/0.6.0/local_installers/cudss-local-repo-ubuntu2404-0.6.0_0.6.0-1_amd64.deb && \
        dpkg -i cudss-local-repo-ubuntu2404-0.6.0_0.6.0-1_amd64.deb && \
        cp /var/cudss-local-repo-ubuntu2404-0.6.0/cudss-*-keyring.gpg /usr/share/keyrings/ && \
        apt-get update && \
        apt-get -y install cudss && \
        rm cudss-local-repo-ubuntu2404-0.6.0_0.6.0-1_amd64.deb; \
    else \
        echo "Skipping CUDSS install in CI environment"; \
    fi

# Install Additional python (only in Dev Environment)
RUN if [ "$CI" = "false" ]; then \
        pip install --no-cache-dir -r /tmp/additional-requirements.txt && rm /tmp/additional-requirements.txt; \
    else \
        echo "Skipping additional package"; \
    fi

# Configure Git
RUN git config --global --add safe.directory '*'

# Set up the working directory
WORKDIR /home/devuser/workspace

# Expose rerun visualization ports
EXPOSE 9876 9877 9090

CMD ["bash"]
