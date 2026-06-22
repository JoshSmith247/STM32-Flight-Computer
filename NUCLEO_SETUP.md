# NUCLEO-H723ZG Bring-Up

Port notes for moving the flight-computer firmware from the custom board to a
**NUCLEO-H723ZG** (Nucleo-144). Same STM32H723ZG die, so the `embassy` feature,
`memory.x`, clock config (runs off **HSI**, not HSE — no crystal change), DMAMUX
request IDs, and all PAC addresses are unchanged.

## Code changes already applied
- **`.cargo/config.toml`** — runner switched to `probe-rs run --chip STM32H723ZGTx`
  (flash + RTT logs over the onboard ST-LINK). Old DFU `flash.sh` kept, commented out.
- **`src/main.rs` panic handler** — GPIOG base corrected `0x4002_1800` → `0x5802_1800`
  (the old value was the STM32F4 base; the panic LED-off never worked on the H7).

## One-time host setup
```sh
cargo install probe-rs-tools     # provides `probe-rs`
cargo run --release              # builds, flashes over ST-LINK, streams defmt logs
```
No BOOT0, no DFU, no power-cycle. `defmt::info!`/`warn!` output appears live in the
terminal — e.g. the IMU WHO_AM_I check, arm/disarm, motor status.

## Hardware: solder bridges to remove (no code change needed)
The firmware keeps its existing pins; resolve the two real conflicts by
**disconnecting the Nucleo's onboard functions** from those pins. Consult the
**UM2407** user-manual solder-bridge (SB) table for the exact SB numbers on your
board revision — remove the bridge that ties each pin to its onboard function:

| Pin | Firmware use | Onboard conflict | Action |
|-----|--------------|------------------|--------|
| **PA7** | SPI1_MOSI (IMU/baro) | Ethernet `RMII_CRS_DV` | Remove the Ethernet SB on PA7 (you don't use Ethernet) |
| **PB0** | Motor M3 (TIM3_CH3) | Green user LED **LD1** | Remove the LD1 SB (or accept LD1 blinking with DSHOT during bench tests) |

Alternative if you'd rather not touch bridges: remap M3 to **PC8** (other TIM3_CH3
pin) — but that needs `motor.rs` changes (GPIOC clock + separate pin config, since
`dshot_init` currently configures all four motor pins on GPIOB), so the SB removal
is cleaner. PA7 has no clean same-instance remap, so the SB removal is preferred there.

## Pins kept as-is (no conflict in practice)
- **USART3 PB10/PB11** (Pi telemetry): the ST-LINK VCP also uses USART3 (on PD8/PD9),
  so the **VCP won't work** — fine, use RTT for logs. The Pi link keeps PB10/PB11.
- **PG7** status LED: free on a morpho header — wire an external LED+resistor, or
  repoint to an onboard LED (PE1/LD2 or PB14/LD3). If you repoint, the onboard LEDs
  are **active-high** (opposite of PG7's active-low), so flip the polarity in
  `status/led.rs` and the init `Level`, and update the panic-handler base/bit.
- **PB4** (Motor M1) is NJTRST by default but free under 2-wire SWD; `dshot_init`
  explicitly sets AF2 so it's overridden.
- **PA2** (USART2_TX) overlaps Ethernet MDIO but is unused by the firmware.

## First-boot checklist on the Nucleo
1. `cargo run --release` → watch defmt logs for init + `WHO_AM_I = 0x47` (IMU found).
2. Confirm it reaches `Idle` (no Fault). With no IMU wired it will Fault — expected.
3. Props **off**: bench motor test (DMAMUX is already `27`, so DSHOT fires).
4. Battery ADC stays disabled until `config.rcc.mux.adcsel` is set (see `main.rs`).
