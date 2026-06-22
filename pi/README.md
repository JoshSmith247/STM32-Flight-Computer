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
rsync -avz --exclude='demo*' /Users/jsmith/Documents/GitHub/STM32-Flight-Computer/pi/ jsmith@raspberrypi.local:~/pi/
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

To check the status once enabled:

```bash
sudo systemctl status mavlink camera
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

## Troubleshooting: is the Pi overloaded?

If the shell lags or hangs after starting the services, the camera/ffmpeg encoder
is the usual cause (software H.264 saturates all 4 cores on a Zero 2 W). Check the
Pi's health right after starting the services:

```bash
top -bn1 | head -12       # ffmpeg at ~360-400% CPU = software encoding, all cores pegged
uptime                    # load average > 4 (it's a 4-core Pi) = overloaded
vcgencmd get_throttled    # 0x0 = OK; nonzero = undervoltage (BEC/power sag)
journalctl -u camera -u mavlink -n 50 --no-pager   # crash-loop / errors
```

The fix is hardware H.264 encoding, which `camera_stream.py` now uses by default
(`ENCODER=hw`). Confirm the Pi actually has the hardware encoder:

```bash
ffmpeg -hide_banner -encoders | grep h264_v4l2m2m   # must print a line
```

If it's missing, set `ENCODER=sw` plus a lower resolution in `~/pi/.env`
(`CAM_WIDTH=640`, `CAM_HEIGHT=480`). After the hw encoder, ffmpeg should drop from
~400% CPU to a fraction, and the shell/network lag should clear.

## WiFi drops constantly (even on a dedicated network)

That's almost always the Pi side, not the access point. Two free checks:

```bash
iw dev wlan0 get power_save        # "on" parks the radio -> random drops
sudo iw dev wlan0 set power_save off
vcgencmd get_throttled             # nonzero = power sag killing WiFi
```

## SSH hangs while the camera streams (WiFi airtime — confirmed cause here)

If the **camera** service hangs your SSH shell but **MAVLink alone doesn't**, and
`top` shows ffmpeg is *not* CPU-bound (i.e. the hw encoder is working), the cause
is **WiFi airtime**: the video stream and your SSH session share the Pi's single
2.4 GHz radio, and the video floods it. Confirm from the laptop while streaming:

```bash
ping <pi-ip>     # latency spiking to 100s of ms / packet loss = airtime saturated
```

The Zero 2 W can't easily put the shell on a separate wire — its one USB data port
is taken by the camera, and the GPIO UART by the STM32. Options:

- **Lower the video bandwidth (best no-cost fix)** in `~/pi/.env`, then drop it
  until `ping` stays responsive while streaming:
  ```
  CAM_BITRATE=800k     # try 800k, then 500k
  CAM_WIDTH=640
  CAM_HEIGHT=480
  CAM_FPS=20
  ```
- **Do SSH work with the camera stopped:** `sudo systemctl stop camera`, work, then
  `sudo systemctl start camera`.
- **Use the local HDMI console** for setup (off WiFi entirely).
- A **Pi 4** fixes this cleanly: wired Ethernet jack for the shell (off WiFi), WiFi
  for video, and a spare USB port for the camera.