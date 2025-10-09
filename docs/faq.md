---
description: Common questions and troubleshooting
---

# FAQ

## GUI apps don’t show up from the container

- Run this on the host before starting: `xhost +local:docker`
- Wayland users: ensure X11 compatibility is enabled, or try an Xorg session

## I started Jazzy but want to switch to Humble

- Stop containers: `docker compose down`
- Start with the humble profile: `./start.sh --profile humble`

## How do I access the container again?

- `docker compose exec ros2_jazzy bash` (or `ros2_humble`)

## Colcon can’t find my packages

- Ensure you’re in the workspace root and the `src/` folder contains packages
- After building, source the overlay: `source install/setup.bash`

## Who do I contact for help?

- Teaching Assistant: Omotoye Adekoya — omotoye.adekoya@edu.unige.it
- Course Professor: Prof. Carmine Recchiuto — carmine.recchiuto@unige.it
