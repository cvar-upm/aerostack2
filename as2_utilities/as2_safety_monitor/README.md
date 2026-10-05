# Safety monitor

Safety monitor node for Aerostack2. It runs the enabled checks and publishes an
`as2_msgs/AlertEvent` on `alert_event` once per violation episode, after the violation has
lasted the check's `seconds`. The alert is published again only after the condition clears.

```bash
ros2 launch as2_safety_monitor as2_safety_monitor_launch.py namespace:=drone0 config_file:=config.yaml
```

Every check is a node parameter block under its name, read from the `safety_monitor` node block
of `config_file` over `config/config_default.yaml`:

- **enabled** (bool): run the check.
- **topic** (string): checked topic, relative to the drone namespace.
- **alert** (int): `AlertEvent` code to publish.
- **description** (string): `AlertEvent` description to publish. The measured state that raised
  the alert goes to the node log.
- **seconds** (float): time the violation must last before the alert, 0 for instant.
- **frequency** (float): evaluation rate of the last message (Hz).

Checks:

- **battery** (`sensor_msgs/BatteryState`): **voltage_threshold** (float), pack voltage the
  alert triggers below.
- **geocage** (`geometry_msgs/PoseStamped`): **polygon** (string), list of at least three
  `[x, y]` vertices in order, e.g. `"[[-3.0, -4.0], [3.0, -4.0], [3.0, 2.5], [-3.0, 2.5]]"`, quoted
  because ROS 2 parameters cannot hold nested lists; **z_min**, **z_max** (float), height limits;
  **frame_id** (string), frame of the cage, `/earth` for the global frame, `map` for
  `<namespace>/map`.

A new check derives from `SafetyCheck<MessageT>`, implements `isViolated()` and `describe()`, and
is added with one `addCheck<>()` line in the node.
