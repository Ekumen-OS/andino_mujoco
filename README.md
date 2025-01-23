# Andino Mujoco simulation

<img src=docs/andino_mujoco.png width=700 />

## :clipboard: Description

This repository contains an [Andino](https://github.com/Ekumen-OS/andino) [MJCF](https://mujoco.readthedocs.io/en/latest/modeling.html) model for use  in a [Mujoco](https://mujoco.readthedocs.io/en/latest/overview.html) simulation, along with some ready-to-use code samples.

## :inbox_tray: Workspace setup

This repository depends only on [Docker](https://www.docker.com/) for running Mujoco and all samples.

## :rocket: Usage

### Mujoco's Python native bindings

This repository provides a base Docker image called `mujoco_python` that includes Mujoco's native [Python bindings](https://mujoco.readthedocs.io/en/latest/python.html). It can be run using the following command:

``` bash
docker compose -f ./docker/docker-compose.yml run --rm mujoco_python
```

### Samples

Samples can be run using the following command (see [`docker/docker-compose.yml`](docker/docker-compose.yml) for the service definition of the samples):

``` bash
docker compose -f ./docker/docker-compose.yml run --rm andino_mujoco_teleoperation
```

## :raised_hands: Contributing

Issues or PRs are always welcome! Please refer to [CONTRIBUTING](CONTRIBUTING.md) doc.