# Hardware datasheets

Reference material for interview prep and bring-up. Downloaded 2026-09-04.

## Downloaded here

| File | Covers |
|---|---|
| `ms5611_baro_datasheet.pdf` | Barometer — pressure/temp conversion sequence, 2nd-order compensation, I2C/SPI register map |
| `qmc5883l_mag_datasheet.pdf` | Magnetometer — register map (CR1/CR2/PERIOD), ODR/range/OSR bit packing, output data format |
| `ublox_m10_product_summary.pdf` | GPS module — electrical specs, pinout, fix types |
| `ublox_m10_ubx_interface_description.pdf` | **The UBX binary protocol itself** — frame format (sync/class/id/length/payload/checksum), NAV-PVT field layout. This is what `gps.rs::parse_pvt` is implementing. |

## Not downloadable from here — vendor sites blocked the automated fetch

These sites (ST, TDK, and TDK's distributor mirrors) return bot-detection blocks to non-browser requests — this isn't a broken link, just something you'll need to grab yourself in an actual browser (takes seconds):

- **STM32H723ZG datasheet** (electrical characteristics, pinout, package):
  https://www.st.com/resource/en/datasheet/stm32h723zg.pdf
- **STM32H723/733 reference manual, RM0468** (the one that actually matters for register-level work — clock tree, DMAMUX, peripheral registers):
  https://www.st.com/resource/en/reference_manual/rm0468-stm32h723733-stm32h725735-and-stm32h730-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
- **ICM-42688-P datasheet** (IMU register map, WHO_AM_I, FIFO, SPI/I2C timing):
  https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000347-icm-42688-p-v1.6.pdf

## Not a formal datasheet — protocol reference instead

- `micolink_protocol_notes.html` — MicoAir's own MICOLINK protocol write-up (frame format, message IDs). No PDF datasheet exists for the MTF-02P; this page is the actual source `flow.rs`'s parser is built against.

## No official spec exists at all

- **SBUS** (RC) — Futaba never published one. What's implemented in `rc.rs` follows the community reverse-engineered format (documented across the Betaflight/ArduPilot/INAV codebases), not a vendor document.
- **MAVLink** (telemetry) — fully open, but it's a live spec at [mavlink.io](https://mavlink.io/en/) / the `common.xml` message dialect, not a single static PDF worth mirroring here.
- **DShot** (motors) — same story as SBUS: community-reverse-engineered (BLHeli/Betaflight ecosystem), no vendor document.
