# Pi Setup

## Pi 4 migration (one-time, on first boot of the Pi 4)

The Pi 4 4GB replaces the Zero 2 W as the dev/bench unit (the Zero 2 W stays as the
flight fallback — same OS, same GPIO/UART, image and config are portable). Everything
in this README applies to both boards. Pi-4-specific steps:

1. **Rename the hostname before both Pis ever share a network** — they both ship as
   `raspberrypi.local`, which collides and makes the SSH host-key dance confusing:
   ```bash
   sudo hostnamectl set-hostname pi4-dev && sudo reboot
   ```
   Then use `ssh jsmith@pi4-dev.local` wherever this doc says `raspberrypi.local`.
2. **Put the shell on Ethernet** — the point of the upgrade. Plug the Pi 4's Ethernet
   into the GL.iNet's LAN port (or directly into the laptop; auto-MDIX + Bonjour make
   `pi4-dev.local` resolve over a direct cable). SSH over Ethernet no longer fights
   the camera stream for WiFi airtime.
3. **`LAPTOP_IP` must be on the video path's network** — with the laptop on two
   interfaces (Ethernet to the Pi's shell + WiFi), `ipconfig getifaddr en0` may return
   the wrong one. Set `LAPTOP_IP` in `.env` to the laptop's IP on whichever network
   the UDP video/telemetry should ride.
4. **Use the 5 V/3 A USB-C supply** on the bench — the Pi 4 browns out on supplies that
   were fine for the Zero 2 W (`vcgencmd get_throttled` ≠ `0x0` = undervoltage).
5. **Optional:** with the shell off WiFi, `CAM_BITRATE` in `.env` can go well above the
   800k cap that protected the Zero 2 W's single radio — raise it once the link is
   proven stable.

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

## First-time setup (run after every reflash)

A fresh OS image has none of these. Run them once on the Pi before installing the
services, or `mavlink`/`camera` will crash-loop.

### 1. System packages

The systemd services run the **system** `/usr/bin/python3` (not the `.venv`), so
`pyserial` must be installed system-wide. `ffmpeg` provides the camera encoder.

```bash
sudo apt update
sudo apt install -y python3-serial ffmpeg rsync
```

### 2. Enable the UART (`/dev/serial0`)

Without this the STM32 serial port doesn't exist and `mavlink.service` fails with
`could not open port /dev/serial0`. `disable-bt` hands the stable PL011 (`ttyAMA0`)
to the GPIO header — needed for reliable 57600-baud MAVLink — and disabling the
serial getty stops the login console from holding the port open.

```bash
# Enable UART + free the PL011 from Bluetooth (idempotent)
sudo sed -i '/^enable_uart=/d; /^dtoverlay=disable-bt$/d' /boot/firmware/config.txt
printf 'enable_uart=1\ndtoverlay=disable-bt\n' | sudo tee -a /boot/firmware/config.txt

# Stop the serial login console from squatting on the port
sudo systemctl disable --now serial-getty@ttyAMA0.service 2>/dev/null
sudo systemctl disable --now serial-getty@serial0.service  2>/dev/null

# Reboot for the overlay to take effect
sudo reboot
```

After it reboots, confirm the port exists before continuing:

```bash
ls -l /dev/serial0      # should symlink → ttyAMA0
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

## Post-reflash tuning (recommended)

Two known-good tweaks for this hardware. Apply them up front rather than waiting to
debug the symptoms (see Troubleshooting below for the full rationale).

- **Cap the camera bitrate** so the video stream doesn't saturate the Zero 2 W's
  single 2.4 GHz radio and lock up your SSH shell. `.env.example` already ships
  `CAM_BITRATE=800k`; confirm it's in your `~/pi/.env` (drop to `500k` if `ping`
  to the Pi still spikes while streaming), then `sudo systemctl restart camera`.
- **Disable WiFi power-save**, which parks the radio and causes random drops. This
  does **not** persist across reboot, so re-run it after every boot (or add it to
  `/etc/rc.local` / a systemd unit):
  ```bash
  sudo iw dev wlan0 set power_save off
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