wget https://developer.download.nvidia.com/compute/cudss/0.6.0/local_installers/cudss-local-repo-ubuntu2404-0.6.0_0.6.0-1_amd64.deb && \
    dpkg -i cudss-local-repo-ubuntu2404-0.6.0_0.6.0-1_amd64.deb && \
    cp /var/cudss-local-repo-ubuntu2404-0.6.0/cudss-*-keyring.gpg /usr/share/keyrings/ && \
    apt-get update && \
    apt-get -y install cudss && \
    rm ./cudss-local-repo-ubuntu2404-0.6.0_0.6.0-1_amd64.deb
