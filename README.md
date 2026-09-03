# Autoware.universe with External Planner
[日本語版READMEはこちら](README_ja.md)

This repository is based on [Autoware.universe](https://github.com/autowarefoundation/autoware_universe) and adds the ability to switch to an external custom planner (`External Planner`).

The goal is to enable incorporation of custom planning algorithms for autonomous driving scenarios that are difficult to handle with the standard Autoware.universe planners such as `LaneDriving` or `Parking`.

> CAUTION
> The `External Planner` implementation itself is not included in this repository and must be implemented separately.

## Overview

Autoware.universe provides various planning capabilities for autonomous driving, supporting scenarios such as lane driving and parking.

However, there are cases where the standard planners cannot handle the required behavior, and a custom planning algorithm tailored to the use case is needed.

This repository adds an `External Planner` scenario to Autoware's planning system so that a custom planner implemented externally can be used.

Switching to the External Planner is done automatically based on the vehicle position and the `external_area` defined on the VectorMap.

### Intended Use Cases
The External Planner mechanism is intended for use in scenarios such as:

- Special autonomous driving maneuvers
- Research and development of new planning algorithms
- Integration of a proprietary planner into Autoware
- Driving scenarios difficult to achieve using Autoware's standard planners

### Node Architecture
![Node Flow](./docs/assets/images/node_flow.png)

## Major Added Features

This repository adds the following features:

- Support for custom planners other than `LaneDriving` and `Parking`
- Automatic switch to an External Scenario based on `external_area`
- Specify the custom planner package to use via ROS parameters
- Safe mode that switches to the External Planner after the vehicle stops
- Functionality to connect the External Planner's trajectory with the `LaneDriving` trajectory
- Smoothing of velocity commands when connecting trajectories
- Completion notification from the External Planner

## Switching to the External Planner

The External Planner is selected based on the `external_area` defined in the VectorMap.

```text
Vehicle
   |
   | approach external_area
   v
+------------------+
| Scenario Selector|
+--------+---------+
         |
         | switch
         v
+------------------+
| External Scenario|
+--------+---------+
         |
         v
+------------------+
| External Planner |
+------------------+
```

The `scenario_selector` searches for `external_area` in the vehicle’s driving direction.

When the vehicle meets the switching conditions, the system switches from the standard planning scenario to the External Scenario.

## Configuration

### Enabling the External Planner

You can enable the External Planner feature from the following launch file:

```text
launch/tier4_planning_launch/launch/scenario_planning/scenario_planning.launch.xml
```

Set the following parameter:

```yaml
use_external: true
```

When using an External Planner, specify the planner package to use with this parameter:

```yaml
external_planner_name: <your_external_planner_package>
```

For example:

```yaml
use_external: true
external_planner_name: my_external_planner
```

If `use_external` is `false`, the External Planner package will not be launched.

### How to configure external_area in the Vector Map

Follow the steps in the link below. 📖[Configuration steps here](./docs/assets/images/README_ExternalArea.md)

### Added Parameters

#### `scenario_selector`

| Parameter                    | Type     | Default | Description                                                                 |
| ---------------------------- | -------- | ------- | --------------------------------------------------------------------------- |
| `search_limit`                 | `double`   | `30.0`    | Search distance [m] for external_area in the driving direction             |
| `area_margin_length`           | `double`   | `0.5`     | Distance [m] added to the LaneDriving trajectory to ensure the vehicle enters external_area |
| `th_old_trajectory_time_sec`   | `double`   | `30.0`    | Valid time [s] of the existing trajectory used when connecting LaneDriving trajectory to External trajectory |
| `use_safe_mode`                | `bool`     | `true`    | If `true`, switch to External Planner after vehicle stops                    |
| `use_external`                 | `bool`     | `true`    | Enable/disable External Planner functionality                               |
| `use_external_extend`          | `bool`     | `false`   | Connect LaneDriving trajectory to the External Planner's trajectory        |
| `use_smooth_extend`            | `bool`     | `false`   | Smooth velocity commands when connecting trajectories                      |

#### scenario_planning.launch.xml

| Parameter               | Type     | Default            | Description                                |
| ----------------------- | -------- | ------------------ | ------------------------------------------ |
| `use_external`            | `bool`     | `false`              | If `true`, launch the External Planner package |
| `external_planner_name`   | `string`   | `external_planner`   | The package name of the External Planner    |

## Dependencies
Message definitions required for integration with the External Planner are needed separately.

Please refer to the following pull request:

- [Toyota/tier4_autoware_msgs#1](https://github.com/Toyota/tier4_autoware_msgs/pull/1)

## Installation

For basic environment setup, follow the standard Autoware.universe installation instructions.

After setting up the Autoware.universe environment, build this repository and any necessary dependency repositories.

For building and installing Autoware.universe, see:

- Autoware Documentation: https://autowarefoundation.github.io/autoware-documentation/
- Autoware Universe Documentation: https://autowarefoundation.github.io/autoware_universe/

## License

This project follows the original Autoware.universe project license. See LICENSE for details.

## Contribution

Thank you for your interest in this project.
We are currently preparing the structure and guidelines to accept external pull requests (planned to start within 2026).
In the meantime, please report bugs or request features via Issues.

## Development & Maintenance Members

This project is currently developed and maintained by:
- Yasuaki Miyahara (Toyota Motor Corporation)
- Naoya Hashimoto (Toyota Motor Corporation)
- Shun Takahashi (Toyota Motor Corporation)
- Daichi Tanizaki (Toyota Motor Corporation)

## Contact

Please open an Issue for bug reports or feature requests.
We will review and address them as much as possible.