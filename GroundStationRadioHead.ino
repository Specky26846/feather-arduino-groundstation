// F' Ground Station bridge for Adafruit Feather M0 RFM69 (915 MHz).
// USB CDC <-> RFM69 packet radio with Space Packet aggregation on uplink.
// RadioHead provides SPI/register access; this sketch owns RFM69's 255-byte
// packet mode because RadioHead's normal send()/recv() API is limited to 60 B.
#include <RH_RF69.h>
#include <zlib.h> // for CRC32 checksum calculation function

namespace {
enum : uint8_t { CS = 8, DIO0 = 3, RST = 4, LED = 13, MAX_PACKET = 255,
                 FIFO_PAYLOAD = 65, FIFO_THRESHOLD = 15, PACKET_HEADER = 8, CRC_SIZE = 4,
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
// Use 0xDEADBEEF as the sync word, following F Prime standard
constexpr uint8_t SYNC[] = {0xDE, 0xAD, 0xBE, 0xEF};

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
      {RH_RF69_REG_2D_PREAMBLELSB, 4}, {RH_RF69_REG_2E_SYNCCONFIG, 0x98}, // SyncSize = 3 for 4 bytes, binary 1001 1000 = 0x98
      {RH_RF69_REG_37_PACKETCONFIG1, 0xC0}, // disabled CrcOn because default is CRC16 and we want CRC32
      {RH_RF69_REG_38_PAYLOADLENGTH, MAX_PACKET},
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

void receivePacket() { // recieves a packet from the radio and sends it to the UART serial port if connected

  uint32_t crc = crc32(0L, Z_NULL, 0); // initialize CRC32
  crc = crc32(crc, downlink, downlinkLength); // now we have the CRC32 in LSB (feather M0 is little-endian)

  // convert CRC32 to big-endian byte array for transmission
  uint8_t crc_bytes[4]; // 4 bytes of 8 bits each (32 total bits)
  crc_bytes[0] = (crc << 24) & 0xFF; // last byte becomes first byte
  crc_bytes[1] = (crc << 16) & 0xFF;
  crc_bytes[2] = (crc << 8) & 0xFF;
  crc_bytes[3] = crc & 0xFF; // in Big Endian now

  uint8_t len_byte[4] = {0, 0, 0, 0}; // 4 bytes of 8 bits each (32 total bits)
  len_byte[3] = downlinkLength; // last byte of the length is the actual length of the payload

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
    // I have the sync character, length byte, and CRC32 bytes. Now send in packet order: sync, length, payload, CRC32.
    if (Serial.dtr()) { // if the USB serial is connected, send the data
      Serial.write(SYNC, sizeof(SYNC)); // send sync character
      Serial.write(len_byte, 4); // send length byte
      Serial.write(downlink, downlinkLength); // send payload
      // may want to move CRC32 calculation here
      Serial.write(crc_bytes, 4); // send CRC32 bytes (transformed to Big Endian)
      restartRx();
    }
  }
}

// Parse custom packet length field (byte 7, last byte of 4-byte length field):
// Format: [SYNC(4)][LENGTH(4)][PAYLOAD(N)][CRC32(4)]
// LENGTH field: bytes 4-7, only byte 7 contains payload length (bytes 4-6 are zeros)
// Total packet size = SYNC + LENGTH + PAYLOAD + CRC32 = 4 + 4 + N + 4 = 12 + N
uint16_t parsePacketLength(const uint8_t* pkt) {
  const uint8_t payloadLen = pkt[7];  // Last byte of LENGTH field
  return static_cast<uint16_t>(PACKET_HEADER + payloadLen + CRC_SIZE);
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

  // Aggregate if you encounter a 0xDEADBEEF sync word, if not drop the byte and continue
  while (uplinkLength >= PACKET_HEADER) {  // reading in just the header
    if (uplink[0] != SYNC[0] || uplink[1] != SYNC[1] ||
        uplink[2] != SYNC[2] || uplink[3] != SYNC[3]) {
      shiftUplink(1);
      continue;
    }

    const uint16_t total = parsePacketLength(uplink);
    // if the total length is less than the header + CRC, not a real packet, drop the first byte and continue
    if (total < PACKET_HEADER + CRC_SIZE) {  
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

    if (uplinkLength < total) break; // if full packet has not arrived yet, break and wait

    if (!sendPacket(uplink, static_cast<uint8_t>(total))) { // transmit - if transmission fails, drop the packet and continue
      shiftUplink(static_cast<uint8_t>(total));
      break;
    }
    
    shiftUplink(static_cast<uint8_t>(total)); // after success transmit, drop the packet from uplink buffer
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
