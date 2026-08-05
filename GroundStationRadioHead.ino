// F' Ground Station bridge for Adafruit Feather M0 RFM69 (915 MHz).
// USB CDC <-> RFM69 packet radio with Space Packet aggregation on uplink.
// RadioHead provides SPI/register access; this sketch owns RFM69's 255-byte
// packet mode because RadioHead's normal send()/recv() API is limited to 60 B.
#include <RH_RF69.h>

namespace {
enum : uint8_t { CS = 8, DIO0 = 3, RST = 4, LED = 13, MAX_PACKET = 255,
                 FIFO_PAYLOAD = 65, FIFO_THRESHOLD = 15, SP_HEADER = 6,
                 // FIFO top-up burst during TX: FIFO(66) - threshold(15) - margin,
                 // matching flight Rfm69Radio TX_TOP_UP_CHUNK so refills stay ahead
                 // of the transmitter for packets larger than the 66-byte FIFO.
                 TX_TOP_UP_CHUNK = 48 };
constexpr uint8_t PA1_13_DBM = 0x5F;  // PA1 enabled, output-power code 31.
constexpr uint8_t OCP_NORMAL = 0x1A;
// Max-length packet ~113 ms on air at 19.2 kb/s; leave margin for FIFO stalls.
constexpr uint32_t RX_PACKET_TIMEOUT_US = 200000;
// Drop incomplete USB Space Packets after this idle gap (ms).
constexpr uint32_t USB_IDLE_FLUSH_MS = 100;
// After TX, wait for RF turnaround before the next uplink SP (ms).
// Align with GDS file-uplink-cooldown (~0.25s) so flight can RX before next SP.
constexpr uint32_t POST_TX_GAP_MS = 50;
// Overflow holding buffer for USB bytes that arrive while uplink[] is full / TX busy.
constexpr uint16_t USB_HOLD_SIZE = 512;
constexpr uint8_t SYNC[] = {0x2D, 0xA7, 0x5C, 0x39, 0xD1, 0x6E, 0x84, 0xF2};

class Radio final : public RH_RF69 {
 public:
  Radio() : RH_RF69(CS, DIO0) {}
  using RH_RF69::spiBurstRead;
  using RH_RF69::spiBurstWrite;
  using RH_RF69::spiRead;
  using RH_RF69::spiWrite;
  bool beginNative() {
    return RHSPIDriver::init() && spiRead(RH_RF69_REG_10_VERSION) == 0x24;
  }
} radio;

uint8_t uplink[MAX_PACKET], downlink[MAX_PACKET];
uint8_t usbHold[USB_HOLD_SIZE];
uint8_t uplinkLength, downlinkLength, downlinkOffset;
uint16_t usbHoldLength;
uint32_t receiveStarted;
uint32_t usbLastByteMs;
uint32_t postTxReadyMs;
bool receiving;

bool mode(uint8_t target) {
  radio.spiWrite(RH_RF69_REG_01_OPMODE,
                 (radio.spiRead(RH_RF69_REG_01_OPMODE) & ~RH_RF69_OPMODE_MODE) | target);
  const uint32_t started = micros();
  while (!(radio.spiRead(RH_RF69_REG_27_IRQFLAGS1) & RH_RF69_IRQFLAGS1_MODEREADY)) {
    if (micros() - started > 20000) return false;
  }
  return true;
}

void restartRx() {
  radio.spiWrite(RH_RF69_REG_3D_PACKETCONFIG2, 0x06);  // AutoRxRestart + RxRestart
  mode(RH_RF69_OPMODE_MODE_RX);
  receiving = false;
  downlinkLength = downlinkOffset = 0;
}

bool configure() {
  digitalWrite(RST, HIGH); delay(10); digitalWrite(RST, LOW); delay(10);
  if (!radio.beginNative()) return false;
  // Match flight defaults: BR_19200 / BW_500_KHZ / DBM_13, 915 MHz, 25 kHz Fdev,
  // FSK-none, sync-byte-0xA7 profile. Reflash when flight DATA_RATE changes.
  const uint8_t cfg[][2] = {
      {RH_RF69_REG_01_OPMODE, RH_RF69_OPMODE_MODE_STDBY},
      {RH_RF69_REG_02_DATAMODUL, RH_RF69_DATAMODUL_DATAMODE_PACKET |
          RH_RF69_DATAMODUL_MODULATIONTYPE_FSK | RH_RF69_DATAMODUL_MODULATIONSHAPING_FSK_NONE},
      // BR_19200 → bitrate register 0x0683 (was BR_9600 = 0x0D05).
      {RH_RF69_REG_03_BITRATEMSB, 0x06}, {RH_RF69_REG_04_BITRATELSB, 0x83},
      {RH_RF69_REG_05_FDEVMSB, 0x01}, {RH_RF69_REG_06_FDEVLSB, 0x9A},
      {RH_RF69_REG_07_FRFMSB, 0xE4}, {RH_RF69_REG_08_FRFMID, 0xC0}, {RH_RF69_REG_09_FRFLSB, 0x00},
      {RH_RF69_REG_11_PALEVEL, PA1_13_DBM},
      {RH_RF69_REG_13_OCP, OCP_NORMAL},
      {RH_RF69_REG_19_RXBW, 0xE0}, {RH_RF69_REG_1A_AFCBW, 0xE0},
      {RH_RF69_REG_26_DIOMAPPING2, RH_RF69_DIOMAPPING2_CLKOUT_FXOSC_OFF},
      {RH_RF69_REG_29_RSSITHRESH, 0xE4}, {RH_RF69_REG_2C_PREAMBLEMSB, 0},
      {RH_RF69_REG_2D_PREAMBLELSB, 4}, {RH_RF69_REG_2E_SYNCCONFIG, 0xB8},
      {RH_RF69_REG_37_PACKETCONFIG1, 0xD0}, {RH_RF69_REG_38_PAYLOADLENGTH, MAX_PACKET},
      {RH_RF69_REG_3C_FIFOTHRESH, 0x80 | FIFO_THRESHOLD},
      {RH_RF69_REG_3D_PACKETCONFIG2, 0x02},
      {RH_RF69_REG_5A_TESTPA1, RH_RF69_TESTPA1_NORMAL},
      {RH_RF69_REG_5C_TESTPA2, RH_RF69_TESTPA2_NORMAL},
      {RH_RF69_REG_6F_TESTDAGC, 0x30},
  };
  for (const auto& reg : cfg) radio.spiWrite(reg[0], reg[1]);
  radio.spiBurstWrite(RH_RF69_REG_2F_SYNCVALUE1, SYNC, sizeof(SYNC));
  radio.spiWrite(RH_RF69_REG_28_IRQFLAGS2, RH_RF69_IRQFLAGS2_FIFOOVERRUN);
  return mode(RH_RF69_OPMODE_MODE_RX);
}

void drainUsbToHold() {
  while (Serial.available() && usbHoldLength < USB_HOLD_SIZE) {
    usbHold[usbHoldLength++] = static_cast<uint8_t>(Serial.read());
    usbLastByteMs = millis();
  }
}

bool sendPacket(const uint8_t* data, uint8_t length) {
  if (!length || !mode(RH_RF69_OPMODE_MODE_STDBY)) return false;
  radio.spiWrite(RH_RF69_REG_28_IRQFLAGS2, RH_RF69_IRQFLAGS2_FIFOOVERRUN);
  radio.spiWrite(RH_RF69_REG_00_FIFO, length);
  uint8_t offset = length < FIFO_PAYLOAD ? length : FIFO_PAYLOAD;
  radio.spiBurstWrite(RH_RF69_REG_00_FIFO, data, offset);
  if (!mode(RH_RF69_OPMODE_MODE_TX)) { restartRx(); return false; }
  const uint32_t started = millis();
  while (millis() - started < 1000) {
    // Do NOT drain USB here: while transmitting a >66 B packet the FIFO must be
    // topped up before it underruns (~21 ms of airtime for the 51 bytes above
    // FifoThreshold at 19.2 kb/s). Reading the next chunk's queued USB bytes
    // one-by-one can stall past that deadline, emptying the FIFO mid-packet and
    // truncating/corrupting the transmission. USB bytes stay buffered by the CDC
    // stack and are drained in receiveUsb() after TX completes.
    const uint8_t flags = radio.spiRead(RH_RF69_REG_28_IRQFLAGS2);
    // Top up whenever the FIFO has drained below FifoThreshold, matching the
    // flight Rfm69Manager::transmitPacket loop (TX_TOP_UP_CHUNK burst, not a
    // threshold-sized dribble that cannot stay ahead of the transmitter).
    if ((offset < length) && !(flags & RH_RF69_IRQFLAGS2_FIFOLEVEL)) {
      const uint8_t count = (length - offset < TX_TOP_UP_CHUNK) ? (length - offset) : TX_TOP_UP_CHUNK;
      radio.spiBurstWrite(RH_RF69_REG_00_FIFO, data + offset, count); offset += count;
    } else if (offset == length && (flags & RH_RF69_IRQFLAGS2_PACKETSENT)) {
      restartRx();
      postTxReadyMs = millis() + POST_TX_GAP_MS;
      return true;
    }
  }
  restartRx(); return false;
}

void receivePacket() {
  uint8_t flags = radio.spiRead(RH_RF69_REG_28_IRQFLAGS2);
  if (!receiving) {
    const bool started = (radio.spiRead(RH_RF69_REG_27_IRQFLAGS1) & RH_RF69_IRQFLAGS1_SYNADDRESSMATCH) ||
                         (flags & RH_RF69_IRQFLAGS2_PAYLOADREADY);
    if (!started || !(flags & RH_RF69_IRQFLAGS2_FIFONOTEMPTY)) return;
    downlinkLength = radio.spiRead(RH_RF69_REG_00_FIFO);
    // Variable-length space-packet RF payloads (1..255). Drop empty / overrun.
    if (downlinkLength == 0) { restartRx(); return; }
    receiving = true; downlinkOffset = 0; receiveStarted = micros();
  }
  if (micros() - receiveStarted >= RX_PACKET_TIMEOUT_US) { restartRx(); return; }
  flags = radio.spiRead(RH_RF69_REG_28_IRQFLAGS2);
  if (flags & RH_RF69_IRQFLAGS2_FIFOOVERRUN) { restartRx(); return; }
  uint8_t count = 0;
  if (flags & RH_RF69_IRQFLAGS2_PAYLOADREADY) count = downlinkLength - downlinkOffset;
  else if (flags & RH_RF69_IRQFLAGS2_FIFOLEVEL)
    count = (downlinkLength - downlinkOffset < FIFO_THRESHOLD) ? downlinkLength - downlinkOffset : FIFO_THRESHOLD;
  if (count) {
    radio.spiBurstRead(RH_RF69_REG_00_FIFO, downlink + downlinkOffset, count);
    downlinkOffset += count;
  }
  if (downlinkOffset == downlinkLength) {
    if (Serial.dtr()) Serial.write(downlink, downlinkLength);
    restartRx();
  }
}

// Parse CCSDS Space Packet primary header length field (bytes 4-5):
// packet data length = N-1 octets of user data after the 6-byte header.
// Full on-wire size = 6 + (N-1) + 1 = header.data_len + 7 ... actually
// spacepackets: packet_len property is total bytes including header.
// CCSDS: length field = (total octets in packet) - 7; total = length_field + 7.
uint16_t spacePacketTotalLength(const uint8_t* hdr) {
  const uint16_t lengthField = (static_cast<uint16_t>(hdr[4]) << 8) | hdr[5];
  return static_cast<uint16_t>(lengthField + 7);
}

void shiftUplink(uint8_t drop) {
  if (drop >= uplinkLength) {
    uplinkLength = 0;
    return;
  }
  memmove(uplink, uplink + drop, uplinkLength - drop);
  uplinkLength = static_cast<uint8_t>(uplinkLength - drop);
}

void pullHoldIntoUplink() {
  while (usbHoldLength > 0 && uplinkLength < MAX_PACKET) {
    const uint16_t space = static_cast<uint16_t>(MAX_PACKET - uplinkLength);
    const uint16_t n = (usbHoldLength < space) ? usbHoldLength : space;
    memcpy(uplink + uplinkLength, usbHold, n);
    uplinkLength = static_cast<uint8_t>(uplinkLength + n);
    memmove(usbHold, usbHold + n, usbHoldLength - n);
    usbHoldLength = static_cast<uint16_t>(usbHoldLength - n);
  }
}

void receiveUsb() {
  drainUsbToHold();
  pullHoldIntoUplink();

  // Respect post-TX RF turnaround before starting another uplink packet.
  if (millis() < postTxReadyMs) return;

  // Aggregate until one complete Space Packet is buffered, then TX it.
  while (uplinkLength >= SP_HEADER) {
    const uint8_t version = (uplink[0] >> 5) & 0x7;
    if (version != 0) {
      shiftUplink(1);
      continue;
    }

    const uint16_t total = spacePacketTotalLength(uplink);
    if (total < SP_HEADER) {
      shiftUplink(1);
      continue;
    }

    // Length > RF MTU: cannot buffer/TX this SP. Drop the whole stream rather
    // than shiftUplink(1), which turns one oversize packet into a long desync.
    if (total > MAX_PACKET) {
      uplinkLength = 0;
      usbHoldLength = 0;
      break;
    }

    if (uplinkLength < total) break;
    if (!sendPacket(uplink, static_cast<uint8_t>(total))) {
      shiftUplink(static_cast<uint8_t>(total));
      break;
    }
    shiftUplink(static_cast<uint8_t>(total));
    pullHoldIntoUplink();
    if (millis() < postTxReadyMs) break;
  }

  // Incomplete garbage after idle: drop so a bad stream cannot wedge forever.
  if (uplinkLength > 0 && (millis() - usbLastByteMs) >= USB_IDLE_FLUSH_MS) {
    uplinkLength = 0;
    usbHoldLength = 0;
  }
}
}  // namespace

void setup() {
  pinMode(LED, OUTPUT); pinMode(RST, OUTPUT); digitalWrite(RST, LOW); Serial.begin(115200);
  uplinkLength = downlinkLength = downlinkOffset = 0;
  usbHoldLength = 0;
  receiving = false;
  usbLastByteMs = 0;
  postTxReadyMs = 0;
  while (!configure()) digitalWrite(LED, (millis() / 250) & 1);
}

void loop() { receiveUsb(); receivePacket(); }
