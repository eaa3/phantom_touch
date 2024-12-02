# Phantom Touch Control

Collection packages to interface with the Phantom Touch devices from 3D systems. 
This package has been tested with the Phantom Touch HID.

## Packages overview

```
.
├── phantom_touch_ros2
|   ├── phantom_touch_control    # main ros2 interfaces with the main node for publishing state and controling the haptic device
|   ├── phantom_touch_description          # contains the Phantom Touch URDF description 
|   ├── phantom_touch_msgs          # contains the device message definitions
```


# Launching the system

## Testing if devices can be found

You can run the LIST using the basic Touch setup tool to check if the devices are being detected.
```bash
Touch_HeadlessSetup LIST
```
If successfull, this should list the connected device's serial number.

## Single device

A single device setup can be spawned using the single device launch:

```bash
ros2 launch phantom_touch_control single_device.launch.py
```

## Two device

A for running two devices at the same time you can launch the system as:

```bash
ros2 launch phantom_touch_control two_devices.launch.py
```

## Dual device - Multi-process

A for running two devices at the same time each managed by a different process node you can launch the system as:

```bash
ros2 launch phantom_touch_control dual_device.launch.py
```