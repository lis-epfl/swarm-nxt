FROM ros:humble

RUN apt-get update && apt-get install -y git wget curl gnupg x11-apps 
RUN mkdir /repos && cd /repos && git clone https://github.com/Peize-Liu/PX4-Autopilot
RUN cd /repos/PX4-Autopilot && \
    ./Tools/setup/ubuntu.sh && \
     make px4_sitl && \
    rm -rf /var/lib/apt/lists/*
