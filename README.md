# ICM20948 Jetson Kernel Module

Out-of-tree kernel module for the ICM20948 IMU used on Jetson platforms.

## Verified Environment
- Jetson Linux: R36.4.7
- Kernel: 5.15.148-tegra
- Board: Jetson Orin Nano (tested)

## Build
```bash
make
```

## Load / Unload
```bash
sudo insmod inv_icm20948.ko
sudo rmmod inv_icm20948
```

## Bind / Unbind I2C Device (example: bus 7, addr 0x68)
```bash
sudo sh -c 'echo icm20948 0x68 > /sys/bus/i2c/devices/i2c-7/new_device'
sudo sh -c 'echo 0x68 > /sys/bus/i2c/devices/i2c-7/delete_device'
```

## Notes
- If `insmod` hangs or I2C times out, reboot to reset the I2C bus.
- This repo contains only the module sources; build artifacts are ignored by `.gitignore`.
