# ROS2-Docker-Exercise-Cross-Language-Temperature-Monitor

Minimal distributed ROS 2 example: a Python publisher that simulates temperature readings and a C++ subscriber that processes 10-sample batches, logs statistics and warnings. The project includes Dockerfiles, a docker-compose configuration, a simple static HTML info page, and optional ROS 2 launch helpers.

---

## Repo layout (important files)
- `src/` — source workspaces and packages
  - `sensor_interface/` — custom message: `TempData.msg`
  - `python_ws/` — Python publisher package
  - `cpp_ws/` — C++ subscriber package
- `docker-compose.yml` — builds and runs both containers
- `index.html` — minimal static status/info page (web)
- `launch/` or `src/.../launch/` — (optional) ROS 2 launch files that call docker commands
- `README.md` — (this file)

---

## Requirements
- Ubuntu (tested with ROS 2 Humble)
- ROS 2 installed (e.g. `ros-humble-desktop`)
- Docker & docker-compose (or Docker engine with `docker compose`)
- `colcon` for building ROS workspaces (if building locally)

Recommend adding your user to the `docker` group:
```bash
sudo usermod -aG docker $USER
newgrp docker
```

---

## Build & run (native ROS 2, no Docker)
1. Source ROS 2:
```bash
source /opt/ros/humble/setup.bash
```
2. From repo root build both workspaces:
```bash
cd /home/openiot/ROS2-Docker-Exercise-Cross-Language-Temperature-Monitor/src/python_ws
colcon build --symlink-install
source install/setup.bash

cd /home/openiot/ROS2-Docker-Exercise-Cross-Language-Temperature-Monitor/src/cpp_ws
colcon build --symlink-install
source install/setup.bash
```
3. Run nodes:
```
# Terminal 1
ros2 run temp_publisher temp_publisher

# Terminal 2
ros2 run temp_subscriber temp_subscriber
```

---

## Run with Docker (recommended for isolation)
1. Make sure `docker-compose.yml` has correct `dockerfile` paths (e.g. `src/python_ws/Dockerfile`) and that any `COPY` in Dockerfiles references the correct relative paths.
2. If your Dockerfiles copy `entrypoint.sh`, ensure files exist at the referenced path (e.g. `src/cpp_ws/entrypoint.sh`).
3. Build & run:
```bash
# from repo root
docker compose build --no-cache
docker compose up --remove-orphans
# or detached
docker compose up -d --remove-orphans
```
4. Stop:
```bash
docker compose down
```

Common docker issues & fixes:
- `failed to read dockerfile: is a directory` — specify `dockerfile: src/.../Dockerfile` (include filename).
- `COPY entrypoint.sh /entrypoint.sh: not found` — ensure `entrypoint.sh` is in build context relative to `context:` or update `COPY` to `COPY src/.../entrypoint.sh /entrypoint.sh`.
- `'ContainerConfig' KeyError` or orphan container problems — remove orphans and prune:
```bash
docker compose down --remove-orphans
docker container prune -f
docker image prune -a -f
docker volume prune -f
docker network prune -f
```

---

## Web page
A static info page `index.html` is provided. To serve locally:
```bash
cd /home/openiot/ROS2-Docker-Exercise-Cross-Language-Temperature-Monitor
python3 -m http.server 8000
# Open http://localhost:8000/index.html
```
Note: current page is static and shows topics/commands. To display live ROS data you'd run ROSBridge and use `roslibjs` (not included in this minimal repo).

---

## Launching Docker from a ROS 2 launch file (learning)
- Place a Python launch file in a package `launch/` directory (e.g. `src/cpp_ws/src/temp_subscriber/launch/docker_containers.launch.py`).
- Use `ExecuteProcess` to run `docker compose` or `docker run`.
- Example actions: build, `docker compose up -d`, wait, then optionally launch ROS nodes locally.

Remember to add launch folder to your package install rules in `CMakeLists.txt`:
```cmake
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
```

---

## QoS events (brief)
The subscriber can register QoS event callbacks to monitor runtime issues:
- `incompatible_qos` — endpoints disagree on QoS (reliability, durability).
- `deadline_missed` — publisher missed configured deadline.
- `liveliness_lost` — publisher lost liveliness.
- `message_lost` — message dropped due to resource limits.

Attach callbacks in C++ with `SubscriptionEventCallbacks` and `SubscriptionOptions.event_callbacks` or for publishers with `PublisherEventCallbacks`.

---

## Useful commands
- List nodes: `ros2 node list`
- List topics: `ros2 topic list`
- Echo topic: `ros2 topic echo /temp_publisher`
- Show interface: `ros2 interface show sensor_interface/TempData`
- Build docker images: `docker compose build`
- Start containers: `docker compose up`

---

## Troubleshooting quick list
- If subscriber doesn't receive messages:
  - Verify topics: `ros2 topic list`
  - Verify message type: `ros2 interface show sensor_interface/TempData`
  - Check `ROS_DOMAIN_ID` consistency between containers/nodes
  - Check QoS mismatch (e.g., publisher `best_effort` vs subscriber `reliable`)
- If docker build fails:
  - Check `dockerfile` paths and `COPY` sources
  - Ensure entrypoint scripts exist and are executable
  - Clean docker caches if needed (`--no-cache`)

---

## Development notes
- Keep `window_size_` and other constants initialized in node constructors to avoid compilation errors.
- Use `std::isnan()` / `std::isinf()` with `#include <cmath>` to validate float values.
- Use `RCLCPP_DEBUG/INFO/WARN/ERROR/FATAL` for appropriate logging.

---

Place issues or questions in repository issues.  