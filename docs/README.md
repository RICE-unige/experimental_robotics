# Experimental Robotics Documentation

> [!warning]
> This space is still being prepared. Students will be notified by the teaching staff once course materials are officially published.

This GitBook collects the practical information you need to work with the Experimental Robotics development environment hosted in this repository. It focuses on reproducible setup, container workflows, simulation tips, and references you can rely on today.

## What You Can Do Here

- Prepare your workstation and launch the Docker-based ROS 2 environment
- Follow streamlined setup checks before touching the robots or simulators
- Troubleshoot common container, GPU, and display issues
- Jump to curated references for ROS 2, Gazebo, Webots, and tooling

## Quick Start Commands

Clone the repository and start with the default ROS 2 Jazzy container:

```bash
git clone https://github.com/RICE-unige/experimental_robotics.git
cd experimental_robotics
./run.sh dev jazzy
```

Need something different? Switch distributions or launch simulators with the same helper script.

{% tabs %}
{% tab title="ROS 2 Jazzy Dev" %}
```bash
./run.sh dev jazzy          # Start Jazzy workspace
docker compose exec dev_jazzy bash
```
{% endtab %}

{% tab title="ROS 2 Humble Dev" %}
```bash
./run.sh dev humble         # Start Humble workspace
docker compose exec dev_humble bash
```
{% endtab %}

{% tab title="Gazebo Simulation" %}
```bash
./run.sh sim rosbotxl gazebo --slam --nav
./connect.sh sim rosbotxl   # Attach to running simulator
```
{% endtab %}
{% endtabs %}

## Handy Shortcuts

- `./connect.sh dev` attaches to the active development container without specifying a name.
- `./run.sh stop all` tears down every container spawned by the helper scripts.
- Export `NVIDIA_VISIBLE_DEVICES=void` before running `run.sh` if you want to force CPU-only rendering.

## Where to Go Next

- [Getting Started](getting-started/setup.md) — workstation prerequisites and validation steps
- [GPU Setup](reference/gpu-setup.md) — optional acceleration for Gazebo and Webots
- [FAQ](reference/faq.md) — quick answers for the most common issues
- [Resources](reference/resources.md) — curated reading and learning material

## Support

- Teaching Assistant: [omotoye.adekoya@edu.unige.it](mailto:omotoye.adekoya@edu.unige.it)
- Course Professor: [carmine.recchiuto@unige.it](mailto:carmine.recchiuto@unige.it)
- Technical issues: open a [GitHub issue](https://github.com/RICE-unige/experimental_robotics/issues)
