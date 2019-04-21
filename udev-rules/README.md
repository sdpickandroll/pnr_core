## UDev Rules

Add these to your `/etc/udev/rules.d/` folder and the PNR devices will be mounted to the following symlinks in the `/dev/` folder:
Quadstick => `quadstick`
Xbox controller => `xbox`
Xbox Adaptive Controller => `xbox_adaptive`

The 3D mouse will be accessible to the `spacenav` node in `plugdev`.