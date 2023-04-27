# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/).

## Unreleased
### Added
- Automatically load configuration when the parameter value changes.
- ROS1 entrypoint is run from shell to ensure correct Python version.
- Parameter `~use_autoware` to publish the Trajectory in `autoware_auto_msgs/Trajectory` format.
- `autoware_auto_msgs` dependency.
- Parameter `~use_internal_points` to use points defined in the configuration file instead of ROS messages.

### Changed
- ROS2 entrypoint is enabled.

## 0.1.0 - 2022-03-14
### Added
- First public version of the package.
