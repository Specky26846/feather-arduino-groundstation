# F' RFM69 Ground Station

Arduino bridge for an **Adafruit Feather M0 RFM69** used as the sole ground
station for the soak / RFM69 packet-radio path.

Uses Adafruit's [RadioHead fork](https://github.com/adafruit/RadioHead) (git
submodule under `libraries/RadioHead`) for SPI / register access only — not
`RH_RF69::send()` / `recv()` (60-byte limit + RadioHead headers). This sketch
owns native RFM69 variable-length packet mode (1–255 B).

## Clone

```bash
git clone --recurse-submodules git@github.com:moisesmata/feather-arduino-groundstation.git
cd feather-arduino-groundstation
```

If you already cloned without submodules:

```bash
git submodule update --init --recursive
```

## Hardware

| Item | Value |
| --- | --- |
| Board | Adafruit Feather M0 RFM69HCW Packet Radio (915 MHz) |
| USB | CDC serial at 115200 baud to GDS / host |
| Pins | CS=8, DIO0=3, RST=4 (Feather RFM69 defaults) |

## Link contract

| Direction | Behavior |
| --- | --- |
| USB → RF (uplink) | Accumulate UART bytes until one complete **CCSDS Space Packet** is buffered (parse 6-byte header length), then TX that packet as one native RFM69 payload (1–255 B). No pad-to-255. Oversized SP length (>255) drops the stream (no byte-shift desync). USB is drained into a hold buffer during TX; post-TX gap paces back-to-back packets. |
| GDS file uplink | Use `--file-uplink-chunk-size 200` and `--file-uplink-cooldown 0.25` so DATA Space Packets fit under 255 B and match RF airtime at 19.2 kb/s. |
| RF → USB (downlink) | Accept any RF payload length 1–255; write those bytes to USB when the packet completes. |
| GDS | Stock `raw-space-packet` framing over UART — **no custom GDS plugin**. |

| Flight configuration | Ground register image |
| --- | --- |
| `DATA_RATE=BR_19200` | `RegBitrate=0x0683` (19,200 b/s). Reflash if flight rate changes. |
| `BANDWIDTH_RX=BW_500_KHZ` | `RegRxBw=RegAfcBw=0xE0` |
| `TX_POWER=DBM_13` | `RegPaLevel=0x5F` (PA1 +13 dBm) |
| Fixed profile | 915 MHz, 25 kHz Fdev, FSK-none, sync `2D A7 5C 39 D1 6E 84 F2` |

## Prerequisites

1. [Arduino CLI](https://arduino.github.io/arduino-cli/) **or** Arduino IDE 2.x
2. Adafruit SAMD board package (`adafruit:samd`)
3. Submodule checked out (`libraries/RadioHead` present)

### One-time arduino-cli setup

```bash
arduino-cli config init   # if you have not already
arduino-cli core update-index
arduino-cli core install adafruit:samd
```

FQBN: `adafruit:samd:adafruit_feather_m0`

## Build / flash (arduino-cli)

From the repo root (so `--library libraries/RadioHead` resolves):

```bash
# Compile
arduino-cli compile \
  --fqbn adafruit:samd:adafruit_feather_m0 \
  --library libraries/RadioHead \
  .

# Flash (replace PORT, e.g. /dev/cu.usbmodem* on macOS)
arduino-cli upload \
  --fqbn adafruit:samd:adafruit_feather_m0 \
  --port PORT \
  --library libraries/RadioHead \
  .
```

List ports with `arduino-cli board list`.

## Build / flash (Arduino IDE)

1. Install **Adafruit SAMD Boards** via Boards Manager.
2. Select **Tools → Board → Adafruit Feather M0**.
3. Either:
   - **Sketch → Include Library → Add .ZIP Library…** pointing at `libraries/RadioHead`, or
   - Set your sketchbook location to this repo root so `libraries/RadioHead` is picked up automatically (**File → Preferences → Sketchbook location**).
4. Open `GroundStationRadioHead.ino` and Upload.

## Layout

```
.
├── GroundStationRadioHead.ino   # USB CDC ↔ RFM69 Space Packet bridge
├── libraries/
│   └── RadioHead/               # submodule: github.com/adafruit/RadioHead
└── README.md
```
