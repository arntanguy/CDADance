# Mosquito Plugin

## Installing

This plugin requires the following dependencies:

```sh
sudo apt install ros-noetic-paho-mqtt-cpp
```

## Testing

1. Start a Mosquitto broker
  ```sh
  mosquitto -p 1883
  ```
2. Run the plugin, make sure the ip is `127.0.0.1` and click `Connect`
3. Check that messages are published with
  ```sh
  mosquitto_sub -h 127.0.0.1 -t robot_stat
  ```
