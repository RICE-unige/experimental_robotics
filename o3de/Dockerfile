FROM husarion/o3de:1.2.0

ENV DEBIAN_FRONTEND=noninteractive

# Install tooling required for VirtualGL + VNC workflow
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        virtualgl \
        xvfb \
        x11vnc \
        fluxbox \
        x11-apps \
        procps \
        ca-certificates && \
    rm -rf /var/lib/apt/lists/*

COPY start.sh /opt/o3de/start.sh
RUN chmod +x /opt/o3de/start.sh

ENV VNC_DISPLAY=":1" \
    VNC_PORT="5901" \
    VNC_RESOLUTION="1920x1080" \
    VNC_DEPTH="24" \
    VGL_DISPLAY=":0" \
    O3DE_EXECUTABLE="Editor" \
    O3DE_ARGS="" \
    O3DE_ASSET_PROCESSOR="1"

CMD ["/opt/o3de/start.sh"]
