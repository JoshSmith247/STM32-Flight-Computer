# Pi Setup

## Host key changed (after reflashing)

After reflashing the SD card, the Pi generates new SSH keys. Clear the old ones on your Mac:

```bash
ssh-keygen -R raspberrypi.local
```

Then reconnect and type `yes` to accept the new fingerprint.

## Connecting via SSH

1. Power on the Pi and wait for it to connect to the hotspot — watch for the connection icon on your phone showing a new device
2. Connect your laptop to the same hotspot
3. SSH in:

```bash
ssh jsmith@raspberrypi.local
```

## Syncing files

If rsync hangs silently, it's likely not installed on the Pi. SSH in and run:

```bash
sudo apt install rsync -y
```

Then run from the repo root on your laptop:

```bash
rsync -avz /Users/jsmith/Documents/GitHub/STM32-Flight-Computer/pi/ jsmith@raspberrypi.local:~/pi/
```

## Installing and starting services

Run once on the Pi after syncing (fixes hardcoded paths, then installs and enables the systemd services):

```bash
sed -i 's|/home/pi/drone|/home/jsmith/pi|g; s|User=pi|User=jsmith|g' ~/pi/mavlink.service ~/pi/camera.service
sudo cp ~/pi/mavlink.service ~/pi/camera.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable mavlink camera
sudo systemctl start mavlink camera
```

If the services fail with "Failed to load environment files", the `.env` file is missing. Create it from the template:

```bash
cp ~/pi/.env.example ~/pi/.env
nano ~/pi/.env
```

Update these values:
- `LAPTOP_IP` — your Mac's IP on the hotspot (e.g. `172.20.10.2`), find with `ipconfig getifaddr en0`.
- `STM32_PORT` — leave as `/dev/serial0` if the STM32 is connected via UART
- Everything else can stay as defaults

Then restart the services:

```bash
sudo systemctl restart mavlink camera
```

Check they're running:

```bash
sudo systemctl status mavlink camera
```

To find the LAN IP address of the ground:

```bash
ipconfig getifaddr en0
```