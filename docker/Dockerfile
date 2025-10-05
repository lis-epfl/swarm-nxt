FROM ros:humble

RUN useradd --user-group --system --create-home --no-log-init --shell /bin/bash lis && \
    usermod -aG sudo lis && echo "lis ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/lis && \
    chmod 440 /etc/sudoers.d/lis
USER lis:lis
WORKDIR /home/lis
RUN sudo apt-get update && sudo apt-get install -y git wget curl gnupg x11-apps 
RUN mkdir /home/lis/repos
RUN git clone --recursive https://github.com/PX4/PX4-Autopilot.git /home/lis/repos/PX4-Autopilot 
RUN cd /home/lis/repos/PX4-Autopilot && \
    git checkout v1.16.0 && \
    git submodule sync --recursive && git submodule update --init --recursive && \
    ./Tools/setup/ubuntu.sh && \
    make px4_sitl
LABEL org.opencontainers.image.source=https://github.com/lis-epfl/swarm-nxt
LABEL org.opencontainers.image.description="SwarmNXT Github Dev Image"
LABEL org.opencontainers.image.licenses=MIT
