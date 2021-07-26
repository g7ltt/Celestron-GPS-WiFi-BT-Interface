/*
 * ESP32 Home-Brew GPS+WiFi+BT+Serial for the Celestron Nexstar AUX bus.
 * Copyright(c) Mark Lord <mlord@pobox.com>.
 * This code is free for personal use/modification/whatever.
 */
#define VERSION "3.7"
#define VERSION_DATE "2021-05-08"
/*
 * Arduino IDE configuration for this project:
 *  -- Remove all use of LED_PIN, in preparation for using SPI bus.
 *  -- First, select Board="ESP32 Dev Module".
 *  -- Now choose Partition Scheme="Huge APP (3MB..)".
 *  -- and also CPU Frequency = 80Mhz (WiFi/BT).
 *
 * Version 3.7
 *  -- Receive/discard echos of packets we send to auxbus, instead of trying auxbus.flush().
 *  -- NexstarGPS: emulate responses for MC_GET_MAX_RATE, MC_GET_MAX_SLEW_RATE command from CPWI.
 * Version 3.6
 *  -- Fix false detection of non-existing GPS receivers.
 *  -- Add 'g' command to toggle GPS debugging
 * Version 3.5
 *  -- Reinstate 15-second connection timeout for recovery from wifi drops.
 *  -- Added automatic management of EVO WiFi as alternative to having a switch for it.
 *  -- More code refactoring to enable use of W5500 ethernet eventually.
 *  -- For Nexstar-GPS mounts, emulate a CPC mount instead.
 *  -- New pin assignments going forward, to eliminate conflicts with SPI/I2C.
 * Version 3.4
 *  -- Doh. Fix bug in GPS-8 compatibility code.
 *  -- Fix corner cases for the new buffering.
 * Version 3.3
 *  -- Implemented buffering of non-0x3b pkt data bytes for WiFi/BT transmission.
 *  -- Fixed harmless (?) bug with Starsense 0x3c data handling.
 *  -- Work on compatibility for older mounts such as the GPS-8.
 * Version 3.2
 *  -- Fix BT SSID generation, and tidy up WiFi mac access/use everywhere.
 * Version 3.1
 *  -- Revamp w2000/w3000 handling using globals for WiFiClient objects.
 *  -- Get rid of auxbus_over_usbserial: nobody wants/uses it.
 *  -- Lots of code revamping/tidying.
 * Version 3.0
 *  -- Remove StarSense Simulator: no longer needed.
 *  -- use "#if BT_ENABLED" instead of "if (BT_ENABLED)"
 * Version 2.9
 *  -- Code cleanups and some restructuring.
 * Version 2.8
 *  -- Add a hyphen into the BT_ID.
 *  -- Add StarSense Camera Simulator
 *  -- Apply various fixes to make WiFi+BT work with StarSense Camera
 * Version 2.7
 *  -- Enable Bluetooth (BT) by default: it works(!) with CPWI.
 *  -- Generate unique BT id based on WiFi hardware mac address.
 * Version 2.6
 *  -- Update auxbus_receive() to better handle StarSense Camera 0x3c packets.
 * Version 2.5
 *  -- Allow for arbitrary auxbus protocols and arbitrary auxbus message sizes.
 * Version 2.4
 *  -- Allow for auxbus messages with up to the protocol limit of 256 bytes payload
 *  -- Get rid of unnecessary rxbuf->count field.
 * Version 2.3
 *  -- Fix GPS pin confusion and add an explanation.
 * Version 2.2
 *  -- Move VERSION to the end of the ADAPTER_VERSION string.  Print it at startup.
 *  -- Change BUSYOUT-to-tx delay to 100 microseconds.
 *  -- Get rid of the verbose=0/1 messages.
 * Version 2.1c
 *  -- New usbserial commands: 't' send test msg; 'v' toggle verbose.
 *  -- Add BUSY diagnostics when verbose=1.
 *  -- Increase BUSYOUT-to-tx delay to 500 microseconds.
 * Version 2.1
 *  -- Fix LED so that it is only on when there's a WiFi connection.
 *  -- Fix parser error on w3000.
 * Version 2.0
 *  -- Get rid of w2000 timeouts: don't seem to be needed after all.
 *  -- Disable Bluetooth by default.
 *  -- Prepend millisecond timestamps to print_packet() output.
 *  -- Replace BUILD_BT flag with BT_ENABLED flag
 *  -- Fixed code use VERBOSE again.
 *  -- Tidying.
 * Version 1.9
 *  -- Make changes to the WiFi mode immediate, both from vars and the physical switch.
 *  -- In the event that the Client parameters are incomplete, it will revert to SoftAP
 *      mode until new vars are "saved".
 * Version 1.8
 *  -- Implemented auxbus_over_usbserial.  Pull pin-19 LOW at startup to activate it.
 *      In theory, this allows for use of MCupdate over the USB-Serial connection.
 *  -- Remap hardware-serial-0 to the GPS, and hardware-serial-1 to USB.
 *      This gets rid of spurious debug messages from Arduino libraries that might mess up auxbus_over_usbserial.
 *  -- Reset the w2000_rxbuf on new connection.
 *  -- Added a (disabled) "version" command on port 3000, same as Celestron.  Enables CFM to detect us.
 * Version 1.7
 *  -- Try to arrange wifi code more sensibly.
 *  -- Added a "verbose" variable on port 3000.
 *  -- Use a (likely) unique default SSID which includes the final 6-digits of MAC.
 *  -- Corrected Pin-D5 to Pin-5 everywhere.
 *  -- Added wifi "reset" command on port 3000.
 *  -- Tidying.
 * Version 1.6
 *  -- Implemented port 3000 management interface.
 *  -- Implemented get/set for all variables used by SkyPortal.
 *  -- Save/restore variables in flash memory.
 *  -- Implemented WiFi-client mode, using config from wlan variables.
 *  -- Designated Pin-18 for use as "restore defaults" in case of misconfiguration.
 *  -- Designated Pin-5 for WiFi mode switch.
 *  -- Added a variable for "wifi.mode": 0=SoftAP, 1=Client, anything else uses the Pin-5 switch.
 *  -- Don't report GPS capability until after a GPSr is detected on its serial port.
 * Version 1.5
 *  -- Reconfigured the board definition to use "huge_app" partitions layout.
 *  -- General tidying.
 * Version 1.4
 *  -- Added working BlueTooth serial support!
 *  -- Not enough flash memory on the ESP32 to hold everything at once though.
 *  -- Need to select BUILD_* features at compile time now.
 * Version 1.3 
 *  -- SkyPortal app now connects instantly! 
 *  -- Added UDP advertisement broadcasts.
 *  -- Added NULL responses to timed-out "GET_VER" queries.
 *  -- Added TCP KEEPALIVE packets at 2000msec intervals.
 * Version 1.2
 *  -- General tidying.
 *  -- Added WiFi on/off switch for ESP32 wifi, using GPIO15 pin.
 *  -- Added WiFi on/off switch for internal EVO wifi, using GPIO4/D4 pin.
 * Version 1.1
 * Version 1.0
 *  -- Initial release.
 *       GPS+WiFi are both working well. Bluetooth (BT) not yet implemented.
 *  -- SkyPortal has a longish delay at startup with the WiFi (be patient).
 *       This is mainly due to SkyPortal polling for StarSense and Focuser.
 *       The Celestron Wifi responds after 100msec with an empty TCP packet,
 *       and I have yet to figure out how to get the ESP32 to do the same.
 *  -- Shorter TCP timeouts not yet implemented; might not be necessary.
 *  -- Wiring schematic not yet drawn up, but circuit uses a 74HC125 for interfacing.
 *
 * The onboard Blue LED is lit when SkyPortal is connected and communicating.
 * It turns off (normally very briefly) when SkyPortal is waiting for a response,
 * or when SkyPortal is no longer connected.
 */

#define VERBOSE           false   // Can also be toggled at run-time (hit 'v' on serial monitor)
#define BT_ENABLED        false   // When false, Bluetooth code is completely left out
#define ETH_ENABLED       false   // When false, Ethernet code is completely left out
#define USE_ORIGINAL_PINS true    // When true, use original pin assignments; when false, use improved pin assignments

static bool verbose;
static bool gpsverbose =  false;  // Can be toggled at run-time (hit 'g' on serial monitor)

#define ADAPTER_VERSION    DEFAULT_SSID "-AMW007-9.0.0.0, " VERSION_DATE "T12:00:00Z, ESP32-" VERSION
// Celestron reports: "ZENTRI-AMW007-1.2.0.10, 2017-07-28T07:43:27Z, ZentriOS-WL-1.2.0.10, Board:N/A");

#define AUXBUS_TXQ_SIZE     8
#define AUXBUS_PKT_MAX     32    // Max length of a regular 0x3b style auxbus packet

#include <WiFi.h>
#include <lwip/sockets.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include "AsyncUDP.h"
#include "FS.h"
#include "SPIFFS.h"
#include <TinyGPS++.h>

#define DEFAULT_SSID "HomeBrew"

// PIN definitions for this project:
#define AUXBUS_RX_PIN               16  // Serial port 2 RX pin
#define AUXBUS_TX_PIN               17  // TX2.  All-in-one uses 17 for USBHost-INT, and uses 4(D4) here instead.
#define AUXBUS_BUSYOUT_PIN          32  // The RTS (aka. "Busy") line output to Nexstar auxBus
#define AUXBUS_BUSYIN_PIN           35  // The RTS (aka. "Busy") line input from Nexstar auxBus
#define GPS_TX_PIN                  -1  // TX-to-GPS pin. Not used.

// Some of the original PIN choices conflict with SPI and I2C busses.
// The default I2C wants pins 21(SDA), 22(SCL).
// The default SPI wants pins 18(SCK), 19(MISO), 23(MOSI), and SS pins: 5(USBhost), 10(ethernet).

// Keep the original PINs for existing units, but redefine them for going forward.
#if USE_ORIGINAL_PINS
#define AUXBUS_TX_PIN               17  // TX2.  All-in-one uses 17 for USBHost-INT, and uses 4(D4) here instead.
#define EVO_WIFI_OFF_PIN            15  // Internal WiFi: ground to turn on, float high to turn off; undefine for automatic on/off.
#define ESP32_WIFI_OFF_PIN           4  // ESP32 WiFi: ground to turn on, float high to turn off
#define ESP32_WIFI_MODE_PIN          5  // ESP32 WiFi: ground to select client-mode; float high for SoftAP mode (CONFLICT with SPI SS)
#define ESP32_RESTORE_DEFAULTS_PIN  18  // ESP32 WiFi: ground to restore vars defaults at wifi startup (CONFLICT with SPI SCK)
#define GPS_RX_PIN                  23  // RX-from-GPS pin.  GPS's TX pin (white) gets connected here (CONFLICT with SPI MOSI)
#else
#warning "Using modified PIN assignments; will NOT work with existing pre-wired projects!!"
#define AUXBUS_TX_PIN                4  // was 17(TX2) but conflicts with USBHost INT pin
#define ESP32_WIFI_OFF_PIN          39  // ESP32 WiFi: ground to turn on, float high to turn off
#define ESP32_WIFI_MODE_PIN         34  // ESP32 WiFi: ground to select client-mode; float high for SoftAP mode
#define GPS_RX_PIN                  33  // RX-from-GPS pin.  GPS's TX pin (white) gets connected here
#endif

HardwareSerial gpsr(1);      // Hardware serial port 1
HardwareSerial auxBus(2);    // Hardware serial port 2, used for Celestron AUX bus

#define LED_ON               HIGH
#define LED_OFF              LOW

static uint8_t    wifi_mac_addr[6];  // Filled in by setup(), used to create unique SSIDs
static WiFiClient w2000;      // The wifi application port 3000, for controlling the mount
static WiFiClient w3000;      // The wifi management  port 3000, for configuring the adapter

// Various "known" device IDs on Nexstar auxBus:
#define DEV_MAIN            0x01
#define DEV_HC              0x04
#define DEV_HC_PLUS         0x0d
#define DEV_HC_SS           0x0e  /* StarSense Hand Controller */
#define DEV_AZM_MC          0x10
#define DEV_ALT_MC          0x11
#define DEV_FOCUSER         0x12
#define DEV_SKYPORTAL       0x20  /* SkyPortal app */
#define DEV_GPS             0xb0  /* Celestron or homebrew GPS */
#define DEV_SP_WIFI         0xb3  /* SkyPortal Wifi */
#define DEV_SSCAMERA        0xb4  /* StarSense Camera */
#define DEV_EVO_WIFI        0xb5  /* EVO Wifi Device */
#define DEV_BATTERY_PC      0xb6  /* EVO Battery Controller */
#define DEV_CHARGE_PORT     0xb7  /* EVO Charge Port */
#define DEV_MOUNT_LIGHTS    0xbf  /* EVO Mount/Tray Lights */
#define DEV_ESP32           0xe2  /* This ESP32 board */
#define DEV_DUMMY           0xe3  /* dummy address when switching evo wifi on/off */

// get a non-zero timeout:
static inline long get_timeout(unsigned int t)
{
   long m = millis() + t;
   return m ? m : 1;
}

// Compare against current time, handling wraparound:
static inline bool time_after (long a, long b)
{
  return (b - a) < 0;
}
#define time_before(a,b) time_after((b),(a))

static struct rxbuf_s {
  byte len;
  byte csum;
  byte data[AUXBUS_PKT_MAX];
  byte requestor;
  const char *name;
} w2000_rxbuf, auxbus_rxbuf;

#if ETH_ENABLED
static struct rxbuf_s e2000_rxbuf;
static void e2000_tx (byte *data, uint16_t len);
#endif

static bool         esp32_wifi_off = true;
static AsyncUDP     udp;
static char         advertisement[128];
static WiFiServer   W2000(2000);  /* Listen on port 2000 for SkyPortal connections */
static WiFiServer   W3000(3000);  /* Listen on port 3000 management commands */

// Because this module implements multiple devices (GPS+BT+Wifi),
//  it is possible that more than one packet may need to be sent simultaneously
//  over the auxbus.  And the bus itself could be in-use from another device at
//  any point in time.  So outbound packets get queued up here and sent when the
//  bus is available.
struct pkt_s {
  uint16_t len;
  byte     data[AUXBUS_PKT_MAX];
};
static struct pkt_s auxbus_txq[AUXBUS_TXQ_SIZE];
static byte         auxbus_txq_head = 0;  // next slot for adding to queue
static byte         auxbus_txq_tail = 0;  // next slot to take from

static void print_packet (const char *prefix, byte *data, uint16_t len)
{
  Serial.printf("%09lu %s: ", millis(), prefix);
  for (uint16_t i = 0; i < len; ++i)
    Serial.printf("%02x ", data[i]);
  Serial.println("");
}

static void auxbus_tx_enq (bool raw, const byte *buf, uint16_t len)
{
  struct pkt_s *p = &auxbus_txq[auxbus_txq_head];
  byte *data = p->data;
  if (p->len != 0) {
    Serial.println("auxbus tx overflow");
    return;
  }
  if (raw) {
    if (len > sizeof(p->data)) {
      Serial.println("auxbus_tx too large");
      return;
    }
    // Copy raw buf to txq, as-is.
    p->len = len;
    memcpy(data, buf, len);
  } else {
    // Prefix with 0x3b, copy buf to txq, and calculate/append the checksum.
    byte csum = len;
    *data++ = 0x3b;
    *data++ = len;
    while (len--) {
      byte c = *buf++;
      csum  += c;
      *data++ = c;
    }
    *data++ = -csum;
    p->len = data - p->data;
  }
  if (++auxbus_txq_head >= AUXBUS_TXQ_SIZE)
    auxbus_txq_head = 0;
}

static TinyGPSPlus gps;
static bool        gpsr_detected = false;  /* Don't respond as DEV_GPS unless a GPSr is detected */

// GPS related commands over Nexstar auxBus:
#define GPS_GET_LAT         0x01
#define GPS_GET_LONG        0x02
#define GPS_GET_DATE        0x03
#define GPS_GET_YEAR        0x04
#define GPS_GET_SAT_INFO    0x07
#define GPS_GET_RCVR_STATUS 0x08
#define GPS_GET_TIME        0x33
#define GPS_TIME_VALID      0x36
#define GPS_LINKED          0x37
#define GPS_GET_HW_VER      0x55
#define GPS_GET_COMPASS     0xa0
#define DEV_GET_VER         0xfe

#define GPS_HW_VER          0xeb    // home-brew GSPr hardware version, to avoid "mount confusion" in CFM

// Changing these to use GNSS reports.  Should probably make it either/or.
TinyGPSCustom satellitesInView(gps, "GPGSV", 3);
TinyGPSCustom fix3D(gps, "GPGSA", 2);          // 1 = no fix,  2 = 2D fix, 3 = 3D fix
TinyGPSCustom fixQuality(gps, "GPGGA", 6);     // 0 = invalid, 1 = GPS,    2 = DGPS, etc...
TinyGPSCustom fix3DGNSS(gps, "GNGSA", 2);      // 1 = no fix,  2 = 2D fix, 3 = 3D fix
TinyGPSCustom fixQualityGNSS(gps, "GNGGA", 6); // 0 = invalid, 1 = GPS,    2 = DGPS, etc...

// For more details how convert GPS position into 24 bit format,
//  see "NexStar Communication Protocol", section "GPS Commands".
// https://www.nexstarsite.com/download/manuals/NexStarCommunicationProtocolV1.2.zip
static const double GPS_MULT_FACTOR = 46603.37778;  // = 2^24 / 360

// Handle both GNSS and GPS format strings
static int getGpsQuality()
{
  String quality(fixQualityGNSS.value());
  if (quality.length())
    return quality.toInt();
  quality = fixQuality.value();
  if (quality.length())
    return quality.toInt();
  return -1;
}

static void handle_gps_request (byte *buf)
{
  if (!gpsr_detected)
    return;
  byte reply[16], op, len = 3;
  reply[0] = buf[3]; //pdst (DEV_GPS)
  reply[1] = buf[2]; //psrc
  reply[2] = op = buf[4];
  switch (op)
  {
    case GPS_LINKED:
    case GPS_TIME_VALID: {
      byte have_gps_fix = getGpsQuality() > 0;
      reply[len++] = have_gps_fix;
      break;
    }

    case GPS_GET_TIME: {
      reply[len++] = gps.time.hour();
      reply[len++] = gps.time.minute();
      reply[len++] = gps.time.second();
      break;
    }

    case GPS_GET_HW_VER: {
      reply[len++] = GPS_HW_VER;
      break;
    }

    case GPS_GET_YEAR: {
      uint16_t year = gps.date.year();
      reply[len++] = year >> 8;
      reply[len++] = year;
      break;
    }

    case GPS_GET_DATE: {
      reply[len++] = gps.date.month();
      reply[len++] = gps.date.day();
      break;
    }

    case GPS_GET_LAT: {
      int32_t lat = (int32_t) (gps.location.lat() * GPS_MULT_FACTOR);
      uint8_t* latBytePtr = (uint8_t*)&lat;
      reply[len++] = latBytePtr[2];
      reply[len++] = latBytePtr[1];
      reply[len++] = latBytePtr[0];
      break;
    }

    case GPS_GET_LONG: {
      int32_t lng = (int32_t) (gps.location.lng() * GPS_MULT_FACTOR);
      uint8_t* lngBytePtr = (uint8_t*)&lng;
      reply[len++] = lngBytePtr[2];
      reply[len++] = lngBytePtr[1];
      reply[len++] = lngBytePtr[0];
      break;
    }

    case GPS_GET_SAT_INFO: {
      String satellitesInViewString(satellitesInView.value());
      reply[len++] = satellitesInViewString.toInt();
      reply[len++] = gps.satellites.value();
      break;
    }

    case DEV_GET_VER: {
      reply[len++] = 2;  // Version 2.0
      reply[len++] = 0;
      break;
    }

    case GPS_GET_RCVR_STATUS:
    case GPS_GET_COMPASS:
    default:
      return;
  }
  auxbus_tx_enq(false, reply, len);
}

static void init_rxbuf (struct rxbuf_s *buf, const char *name)
{
  memset(buf, 0, sizeof(*buf));
  buf->name = name;
}

enum {pkt_fail, pkt_inprogress, pkt_complete};

static byte last_auxbus_tx[AUXBUS_PKT_MAX] = {0,};
static byte last_auxbus_tx_len = 0;

static inline byte packet_reset (struct rxbuf_s *buf, byte ret)
{
  if (buf == &auxbus_rxbuf)
    last_auxbus_tx_len = 0;
  else
    buf->len = 0;
  return ret;
}

static byte packet_decoder (struct rxbuf_s *buf, byte b)
{
  static uint16_t sscount = 0, sslen = 0;   // Used to crudely track/debug StarSense camera 0x3c packets
  buf->data[buf->len++] = b;
  if (buf->len == 1) {
    if (b != 0x3b) {
      if (verbose)
        Serial.printf("%09u %s: %02x\n", millis(), buf->name, b);
      //buf->len = 0;  /* FIXME: uncomment this for testing with GPS-8 mount */
      if (!sscount) {
        if (b == 0x3c)
          sscount = 1;
      } else {
        if (++sscount == 4)
          sslen = b;
        else if (sscount == 5)
          sslen = ((sslen << 8) | b) + 6;
      }
      return packet_reset(buf, pkt_fail);
    }
    if (sscount)
      Serial.printf("%09u Starsense: %u/%u\n", millis(), sscount, sslen);
    if (sscount == sslen)
      sscount = sslen = 0;
    buf->csum = 0;
    return pkt_inprogress;
  }
  buf->csum += b;
  if (buf->len == 1)
    return pkt_inprogress;
  if (buf->len == 2) {
    if (b < 3 || b >= (sizeof(buf->data) - 5)) {
      if (verbose)
        Serial.printf("%s: bad len %02x\n", buf->name, b);
      return packet_reset(buf, pkt_fail);
    }
    return pkt_inprogress;
  }
  if (buf->len != (buf->data[1] + 3))
    return pkt_inprogress;
  if (buf->csum != 0) {
    if (verbose)
      Serial.printf("%s: bad csum %02x\n", buf->name, buf->csum);
    return packet_reset(buf, pkt_fail);
  }

  // Handle a fully completed packet:
  if (buf->len == last_auxbus_tx_len && buf == &auxbus_rxbuf) {
    if (0 == memcmp(buf->data, last_auxbus_tx, last_auxbus_tx_len)) {
      buf->len = 0;  // echo-back of last auxbus packet we sent: discard it
      return packet_reset(buf, pkt_fail);
    }
  }
  buf->requestor = buf->data[2];
  byte target    = buf->data[3];
  if (verbose || target == DEV_ESP32) {
    if (target == DEV_ESP32)
      Serial.print("Received: ");
    print_packet(buf->name, buf->data, buf->len);
  }
  if (target == DEV_GPS)
    handle_gps_request(buf->data);
  if (buf != &auxbus_rxbuf)
    auxbus_tx_enq(true, buf->data, buf->len);
  return packet_reset(buf, pkt_complete);
}

#if BT_ENABLED
#include "BluetoothSerial.h"
BluetoothSerial        bt;
static struct rxbuf_s  bt_rxbuf;
static bool            bt_connected = false;

static inline void bt_tx (byte *data, uint16_t count)
{
  if (bt_connected) {
    bt.write(data, count);
    if (verbose)
      print_packet(__func__, data, count);
  }
}
#endif /* BT_ENABLED */

static void w2000_tx (byte *data, uint16_t count)
{
  if (!esp32_wifi_off && w2000.connected()) {
    w2000.write(data, count);
    if (verbose)
      print_packet(__func__, data, count);
  }
}

static void insert_checksum (byte *data, uint16_t len)
{
  if (len-- < 6 || data[0] != 0x3b)
    return;
  byte i, csum = data[1];
  for (i = 2; i < len; ++i)
    csum += data[i];
  data[len] = -csum;
}

/* Try and achieve compatibility with older mounts such as the GPS-8 */
static uint16_t hack_response_for_compatibility (byte *data, uint16_t len)
{
#if 1
  // MC_GET_MODEL 1-byte response from a GPS-8 mount: {0x3b,0x04,0x10,0x20,0x05,0x01,0xc6}:
  // SkyPortal expects a 2-byte response to MC_GET_MODEL (0x05).
  if (len == 7 && data[0] == 0x3b && data[2] == 0x10 && data[4] == 0x05) {
    data[1] += 1;
    data[6] = 0x89;  // emulate a CPC mount.  (was: data[6] = data[5];)
    data[5] = 0x11;  // emulate a CPC mount.  (was: data[5] = 0;)
    insert_checksum(data, ++len);
    return len;
  }
#endif
#if 1
  // MC_GET_MAX_RATE response from a GPS-8 mount: {0x3b,0x03,0x10,0x20,0x23,0xaa}:
  // SkyPortal expects a 1-byte response to MC_GET_MAX_RATE (0x23):
  if (len == 6 && data[0] == 0x3b && data[2] == 0x10 && data[4] == 0x23) {
    data[1] += 1;
    data[5] = 0x00;  // return 00 for MC_GET_MAX_RATE, same as Evolution mount
    insert_checksum(data, ++len);
    return len;
  }
#endif
#if 1
  // Emulate MC_GET_MAX_SLEW_RATE for NexstarGPS, using response data from an Evolution mount:
  if (len == 6 && data[0] == 0x3b && data[2] == 0x10 && data[4] == 0x21) {
    data[1] += 4;
    data[5] = 0xa0;
    data[6] = 0x11;
    data[7] = 0x94;
    data[8] = 0x54;
    len += 4;
    insert_checksum(data, len);
    return len;
  }
#endif
#if 0   // for testing only:
  if (len > 5 && data[0] == 0x3b && data[2] == 0x10 && data[4] == 0x05) {
    //static const byte new_response[] = {0x3b,0x04,0x10,0x20,0x05,0x01,0xc6};       // GPS-8 original
    //static const byte new_response[] = {0x3b,0x05,0x10,0x20,0x05,0x11,0x89,0x2c};  // CPC Deluxe
    static const byte new_response[]   = {0x3b,0x05,0x10,0x20,0x05,0x00,0x01,0xc5};  // GPS-8 corrected
    len = sizeof(new_response);
    memcpy(data, new_response, len);
  }
#endif
  return len;
}

static void bridge_auxbus_to_others (byte *data, uint16_t len)
{
  byte requestor, destination;
  bool sscamera_pkt;

  // Try not to be confused by StarSense Camera data:
  if (len < 6 || data[0] != 0x3b || data[1] < 3 || data[2] == 0) {
    sscamera_pkt = true;
    if (!verbose)
      print_packet("Unknown", data, len);
  } else {
    sscamera_pkt = false;
    if (0 == (len = hack_response_for_compatibility(data, len)))
      return;
    requestor   = data[2] ? data[2] : 0xff;
    destination = data[3];
  }

  // Echo packet to w2000 (wifi), unless it came from w2000:
  if (sscamera_pkt || requestor != w2000_rxbuf.requestor)
    w2000_tx(data, len);
  if (!sscamera_pkt && destination == w2000_rxbuf.requestor)
    w2000_rxbuf.requestor = 0;

#if ETH_ENABLED
  // Echo packet to e2000 (ethernet), unless it came from e2000:
  if (sscamera_pkt || requestor != e2000_rxbuf.requestor)
    e2000_tx(data, len);
  if (!sscamera_pkt && destination == w2000_rxbuf.requestor)
    e2000_rxbuf.requestor = 0;
#endif /* ETH_ENABLED */

#if BT_AUX_ENABLED
  // Echo packet to bt, unless it came from bt:
  if (sscamera_pkt || requestor != bt_rxbuf.requestor)
    bt_tx(data, len);
  if (!sscamera_pkt && destination == bt_rxbuf.requestor)
    bt_rxbuf.requestor = 0;
#endif /* BT_AUX_ENABLED */
}

// Buffering Starsense Camera packet data is surprisingly complex and difficult:
static void auxbus_receive ()
{
  static long delay = 0;
  static byte data[AUXBUS_PKT_MAX*2], count = 0;
  struct rxbuf_s *buf = &auxbus_rxbuf;

  while (auxBus.available()) {
    byte status = packet_decoder(buf, auxBus.read());
    if (status == pkt_inprogress) {
      if (buf->len == 1 && count) {
        // New 0x3b packet, so flush anything left over from before:
        bridge_auxbus_to_others(data, count);
        count = 0;
        delay = 0;
      }
    } else if (buf->len) {
      memcpy(data + count, buf->data, buf->len);
      count += buf->len;
      buf->len = 0;  // reset rxbuf for next packet
      if (status == pkt_complete || count >= AUXBUS_PKT_MAX) {
        bridge_auxbus_to_others(data, count);
        count = 0;
        delay = 0;
      } else if (!delay) {
        delay = get_timeout(20);
      }
    }
  }
  if (count && (!delay || time_after(millis(), delay))) {
    bridge_auxbus_to_others(data, count);
    count = 0;
    delay = 0;
  }
}

static inline struct pkt_s *claim_auxbus_for_tx (bool verbose)
{
  static bool was_busy = false;
  struct pkt_s *p = &auxbus_txq[auxbus_txq_tail];

  if (p->len == 0)
    return NULL;  /* Nothing to send; txq is empty. */
  if (digitalRead(AUXBUS_BUSYIN_PIN) == LOW) {
    if (verbose && !was_busy)
      Serial.println("Busy");
    was_busy = true;
    return NULL;  /* auxbus is busy */
  }
  // "Claim" the auxbus
  digitalWrite(AUXBUS_BUSYOUT_PIN, LOW);    // Assert BUSYOUT   
  if (verbose && was_busy)
    Serial.println("Idle");
  was_busy = false;
  return p;  // auxbus claimed for tx
}

static inline bool auxbus_tx (bool verbose, struct pkt_s *p)
{  
  // Transmit the packet
  for (uint16_t i = 0; i < p->len; ++i)
    auxBus.write(p->data[i]);
  auxBus.flush(true);      // flush tx only
#if 0
  // Now try to get rid of the echo of the tx'd packet:
  delayMicroseconds(600);  // 500usec per character at 19200bps
  auxBus.flush(false);     // flush both tx and rx
#endif
  // Release the bus
  digitalWrite(AUXBUS_BUSYOUT_PIN, HIGH);   // Tri-state BUSYOUT

  if (verbose)
    print_packet(__func__, p->data, p->len);
  bridge_auxbus_to_others(p->data, p->len);

  // Free the txq entry and advance the txq_tail index
  p->len = 0;
  if (++auxbus_txq_tail >= AUXBUS_TXQ_SIZE)
    auxbus_txq_tail = 0;
  return true;  // All okay. Sent a packet.
}

static bool service_auxbus (bool verbose)
{
  struct pkt_s *p;
  long min_busy_time;

  auxbus_receive();                              // service/empty the input FIFO
  if (!(p = claim_auxbus_for_tx(verbose)))
    return false;  /* Nothing to send, or bus is busy */
  min_busy_time = micros() + 100;                // Allow BUSY to assert for 100usec before tx
  auxbus_receive();                              // Ensure rx FIFO is clear; it will be wiped after tx
  while (time_before(micros(), min_busy_time));  // Delay tx until BUSY is asserted long enough
  return auxbus_tx(verbose, p);                  // All good; now send the packet
}

struct nvram_variable_s {
  char *name;
  char value[64];
};

#define VARS_COUNT  12
static char *vars[VARS_COUNT] = {
  (char *)"softap.passkey",
  (char *)"softap.ssid",
  (char *)"verbose",
  (char *)"wifi.mode",
  (char *)"wlan.dhcp.enabled",
  (char *)"wlan.passkey",
  (char *)"wlan.ssid",
  (char *)"wlan.static.dns",
  (char *)"wlan.static.gateway",
  (char *)"wlan.static.ip",
  (char *)"wlan.static.netmask",
  (char *)NULL
};

#define VALS_MAX_LEN 31

static char vals[VARS_COUNT][VALS_MAX_LEN + 1];
static const char vals_defaults[VARS_COUNT][VALS_MAX_LEN + 1] = {
  "",               // softap.passkey
  "",               // softap.ssid
  "0",              // verbose
  "Pin-5",          // wifi.mode: "Pin-5", "0" (SoftAP), or "1" (Client)
  "1",              // wlan.dhcp.enabled
  "",               // wlan.passkey
  "",               // wlan.ssid
  "8.8.8.8",        // wlan.static.dns
  "0.0.0.0",        // wlan.static.gateway
  "0.0.0.0",        // wlan.static.ip
  "0.0.0.0",        // wlan.static.netmask
  ""
};

static bool vars_initialized = false;

static bool wifi_client_mode          = false;  // true=client; false=SoftAP
static bool wifi_client_mode_override = false;  // true=client; false=whichever

static void vars_load_defaults ()
{
  Serial.println(__func__);
  memcpy(vals, vals_defaults, sizeof(vals));

  // Set up a unique default SSID by appending last 6-digits of MAC:
  char       *ssid = vals[find_val("softap.ssid")];
  sprintf(ssid, "%s-%02X%02X%02X", DEFAULT_SSID, wifi_mac_addr[3], wifi_mac_addr[4], wifi_mac_addr[5]);
}

static void set_verbose()
{
  char v = *vals[find_val("verbose")];
  verbose = (v == '1') ? true : VERBOSE;
}

static void vars_restore ()
{
  if (vars_initialized)
      return;
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS.begin failed");
  } else {
    vars_initialized = true;
#ifdef ESP32_RESTORE_DEFAULTS_PIN
    if (digitalRead(ESP32_RESTORE_DEFAULTS_PIN) == HIGH)
#endif
    {
      File vf = SPIFFS.open("/nexstar.txt", FILE_READ);
      if (!vf) {
        Serial.println("open(/nexstar.txt) failed");
      } else {
        size_t ret = vf.read((uint8_t *)vals, sizeof(vals));
        vf.close();
        if (ret == sizeof(vals))
          goto loaded;  /* success! */
        Serial.printf("read(/nexstar.txt) failed: %d/%u\n", ret, sizeof(vals));
      }
    }
  }
  vars_load_defaults();
loaded:
  set_verbose();
  // SkyPortal app looks for these UDP broadcasts. "AMW007" seems to be the critical bit.
  // Insert our MAC address and ADAPTER_VERSION string into the advertisement broadcast string:
  sprintf(advertisement, "{\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\",\n\"version\":\"" ADAPTER_VERSION "\"\n}",
    wifi_mac_addr[0], wifi_mac_addr[1], wifi_mac_addr[2], wifi_mac_addr[3], wifi_mac_addr[4], wifi_mac_addr[5]);
  if (verbose)
    Serial.printf("UDP advertisement=%s\n", advertisement);
}

static void vars_save ()
{
  if (!vars_initialized)
    return;
  File vf = SPIFFS.open("/nexstar.txt", FILE_WRITE);
  if (!vf) {
    Serial.println("open(/nexstar.txt) failed");
    return;
  }
  vf.write((uint8_t *)vals, sizeof(vals));
  vf.close();
}

static int find_val (const char *name)
{
  for (int i = 0; vars[i]; ++i) {
    const char *v = vars[i];
    byte len = strlen(v);
    if (0 == memcmp(name, v, len)) {
      if (name[len] == '\0' || name[len] == ' ') {
        return i;
      }
    }
  }
  return -1;
}

#define SERVER_IPADDRESS  1,2,3,4
static const IPAddress server_static_ip(SERVER_IPADDRESS);
static const IPAddress server_static_netmask(255,255,255,240); 

static void wifi_begin_server_mode()
{
  char *ssid    = vals[find_val("softap.ssid")];
  char *passkey = vals[find_val("softap.passkey")];
  if (!ssid[0])
    vars_load_defaults();
  Serial.printf("esp32_wifi ON, softAP, ssid=%s, passkey=%s\n", ssid, passkey);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, passkey);
  delay(100);  // This delay here is critical!!
  WiFi.softAPConfig(server_static_ip, server_static_ip, server_static_netmask);
}

static bool wifi_begin_client_mode()
{
  char *ssid     = vals[find_val("wlan.ssid")];
  char *passkey  = vals[find_val("wlan.passkey")];
  char *use_dhcp = vals[find_val("wlan.dhcp.enabled")];
  if (!ssid[0]) {
    wifi_client_mode_override = true;
    return false;  /* cannot be a client unless we know which AP to connect to! */
  }
  WiFi.mode(WIFI_STA);
  if (use_dhcp[0] != '1') {
    char *ip     = vals[find_val("wlan.static.ip")];
    char *nmask  = vals[find_val("wlan.static.netmask")];
    char *gw     = vals[find_val("wlan.static.gateway")];
    char *dns    = vals[find_val("wlan.static.dns")];
    IPAddress IP, SUBNET, GW, DNS;
    if (IP.fromString(ip) && SUBNET.fromString(nmask) && GW.fromString(gw) && DNS.fromString(dns))
      WiFi.config(IP, DNS, GW, SUBNET);
    else
      Serial.println("static IP config failed; using DHCP");
  }
  WiFi.begin(ssid, passkey);
  while (WiFi.status() != WL_CONNECTED)
    service_non_wifi_devices();
  Serial.printf("\nWiFi connected to %s, IP address: ", ssid);
  Serial.println(WiFi.localIP());
  return true;
}

static bool is_wifi_client_mode_requested()
{
  if (wifi_client_mode_override)
    return false;
  char *wifi_mode = vals[find_val("wifi.mode")];
  if (wifi_mode[0] == '1' || digitalRead(ESP32_WIFI_MODE_PIN) == LOW)
    return true;
  return false;
}

static void set_esp32_wifi (bool want_off)
{
  if (esp32_wifi_off == want_off)
    return;  /* all okay as-is */
  esp32_wifi_off = want_off;
  if (want_off) {
    Serial.println("esp32_wifi OFF");
    if (!wifi_client_mode)
      WiFi.softAPdisconnect(true);
  } else {
    wifi_client_mode = is_wifi_client_mode_requested();
    Serial.printf("esp32_wifi ON, %s mode\n", wifi_client_mode ? "Client" : "Server");
    if (wifi_client_mode)
      wifi_client_mode = wifi_begin_client_mode();
    if (!wifi_client_mode)
      wifi_begin_server_mode();
    W2000.begin();  // Start listening for application connections on wifi
    W3000.begin();  // Start listening for management commands
  }
}

static void manage_esp32_wifi_onoff ()
{
  bool want_off = (digitalRead(ESP32_WIFI_OFF_PIN) == HIGH);
  set_esp32_wifi(want_off);
  if (!want_off) {
    bool want_mode = is_wifi_client_mode_requested();  //FIXME: shouldn't be reading flash so often!
    if (wifi_client_mode != is_wifi_client_mode_requested()) {
      set_esp32_wifi(true);  // turn off WiFi
      set_esp32_wifi(false); // turn on WiFi using the requested mode
    }
  }
}

/*
 * Keep tally of all HomeBrew connections here,
 *  and turn Evolution WiFi off when at least one is connected,
 *  otherwise turn it back on after final disconnection.
 *
 * Note: When setting EVO WiFi off, an extra stray 0x00 is received.
 *   and when setting EVO Wifi on, an extra 0x0a is received from the WiFi.
 */
static void set_evo_wifi_off (bool turn_off)
{
#ifndef EVO_WIFI_OFF_PIN
  static byte connections = 0;
  byte old_connections = connections;
  if (turn_off)
    ++connections;
  else if (connections)
    --connections;
  if ((old_connections && !connections) || (!old_connections && connections)) {
    Serial.printf("evo_wifi %s\n", turn_off ? "OFF" : "ON");
    byte msg[] = {DEV_DUMMY, DEV_EVO_WIFI, 0x10, !turn_off};
    auxbus_tx_enq(false, msg, sizeof(msg));
  }
#endif
}

static void manage_evo_wifi_switch ()
{
#ifdef EVO_WIFI_OFF_PIN
  static bool initialized = false;
  static bool wifi_off = false;
  if (!initialized) {
    pinMode(EVO_WIFI_OFF_PIN, INPUT_PULLUP);
    if (millis() < 7000)
      return;
    initialized = true;
  }
  bool want_off = (digitalRead(EVO_WIFI_OFF_PIN) == HIGH);
  if (wifi_off == want_off)
    return;  /* all okay as-is */
  wifi_off = want_off;
  Serial.printf("evo_wifi %s\n", want_off ? "OFF" : "ON");
  byte msg[] = {DEV_DUMMY, DEV_EVO_WIFI, 0x10, !want_off};
  auxbus_tx_enq(false, msg, sizeof(msg));
#endif
}

/* Test auxbus by sending a message and (elsewhere) seeing the response */
static void send_test_msg ()
{
  byte msg[] = {DEV_ESP32, DEV_AZM_MC, 0xfe};
  bool sent;
  long timeout;

  while (service_auxbus(verbose));  // empty the auxbus tx queue (not perfect..)
  auxbus_tx_enq(false, msg, sizeof(msg));   // enqueue the test message
  timeout = get_timeout(2000);
  do {
    Serial.print("Sending:  ");
    sent = service_auxbus(true);      // send the test message (also prints it)
    if (!sent)
      delay(10);
  } while (!sent && time_before(millis(), timeout));
} 

static void service_non_wifi_devices ()
{
  while (Serial.available()) {
    byte b = Serial.read();
    if (b == 't' || b == 'T')
      send_test_msg();
    else if (b == 'v' || b == 'V')
      verbose = !verbose;
    else if (b == 'g' || b == 'G') {
      gpsverbose = !gpsverbose;
      Serial.printf("gpsverbose=%s\n", gpsverbose ? "on" : "off");
    }
  }
  while (gpsr.available()) {
    char c = gpsr.read();
    // Garbage characters can confuse detection, so qualify with '\r':
    if (!gpsr_detected && c == '\r') {
      gpsr_detected = true;
      Serial.println("gpsr_detected");
    }
    gps.encode(c);
    if (gpsverbose)
      Serial.print(c);
  }
#if BT_ENABLED
  while (bt.available()) {
    bt_connected = true;  // enables bt_tx()
    (void)packet_decoder(&bt_rxbuf, bt.read());
  }
#endif /* BT_ENABLED */
  (void)service_auxbus(verbose);
}

static void w3000_write (const char *s)
{
  if (verbose)
    Serial.printf("w3000: %s\n", s);
  w3000.write(s);
}

/*
 * p3000_decoder() could be used for WiFi or Ethernet
 */
static void p3000_decoder (char b, void (*p3000_write)(const char *))
{
  static char cmd[64], cx = 0;
  if (cx == 0)
    memset(cmd, 0, sizeof(cmd));
  if ((cx < sizeof(cmd) - 1) && b != '\r' && b != '\n')
    cmd[cx++] = b;
  if (b == '\n') {
    if (verbose)
      Serial.printf("p3000: %s\n", cmd);
    if (0 == strcmp(cmd, "save")) {
      vars_save();
      p3000_write("Success");
      wifi_client_mode_override = false;
#if 0  /* Enable this to have CFM recognize the Wifi.. DANGEROUS! */
    } else if (0 == strcmp(cmd, "version")) {
      p3000_write(ADAPTER_VERSION);
      /*
       * CFM connects to port 2000.
       * CFM then sends \r\n and waits 250msecs.
       * CFM disconnects.
       * CFM connects to port 3000, and issues version command.
       * CFM disconnects.
       * CFM connects to port 2000.
       * CFM sends \r\n and waits 200msecs.
       * CFM sends dasEcho and waits 1100msecs.
       * CFM disconnects/reconnects to port 2000.
       * CFM sends \r\n and waits 200msecs.
       * CFM sends          3b 07 20 00 51 00 01 c2 00 c5
       * The mount replies: 3b 03 00 20 51 8c
       * CFM sends: 05 00 06 38 c0 and waits 40msecs.
       * CFM sends: 06 00 02 02 f5 d8 and waits 200msecs.
       * CFM sends: 06 00 02 02 f5 d8 and waits 200msecs again.
       * CFM speaks proper 0x3b protocol from there on.
       */
#endif
    } else if (0 == strcmp(cmd, "reset")) {
      set_esp32_wifi(true); // turn WiFi off; it will turn on again automatically
    } else if (0 == strcmp(cmd, "load defaults")) {
      vars_load_defaults();
      p3000_write("Loaded defaults");
    } else if (0 == strcmp(cmd, "get all")) {
      for (int i = 0; vars[i]; ++i) {
        p3000_write(vars[i]);
        p3000_write(": ");
        p3000_write(vals[i]);
        if (vars[i + 1])
          p3000_write("\r\n");
      }
    } else if ((cmd[0] == 'g' || cmd[0] == 's') && cmd[1] == 'e' && cmd[2] == 't' && cmd[3] == ' '  && cmd[4]) {
      char *c = cmd + 4;
      int x = find_val(c);
      if (x >= 0) {
        if (cmd[0] == 'g') {  // "get"
          p3000_write(vals[x]);
        } else {  // "set"
          c += 1 + strlen(vars[x]);
          byte quoted = (*c == '"');
          c += quoted;
          if (strlen(c) < VALS_MAX_LEN) {
            char *v = vals[x];
            memset(v, 0, sizeof(vals[x]));
            while (*c && (!quoted || *c != '"'))
              *v++ = *c++;
            p3000_write("Set OK");
            if (0 == strcmp(vars[x], "verbose"))
              set_verbose();
          }
        }
      }
    }
    p3000_write("\r\n> ");
    cx = 0;
  }
}

// Common code for service_w2000() and service_w3000():
static bool wifi_port_is_connected (WiFiClient client, bool *connected, void (*poll_for_connection)(), const char *name)
{
  if (esp32_wifi_off) {
    *connected = false;
  } else if (client.connected()) {
    if (!*connected) {
      *connected = true;
      Serial.printf("%09u %s Connected\n", millis(), name);
    }
  } else {
    if (*connected) {
      *connected = false;
      Serial.printf("%09u %s Disconnected\n", millis(), name);
    }
    poll_for_connection();
  }
  return *connected;
}

static void poll_for_new_w3000_connection()
{
  w3000 = W3000.available();
}

static void service_w3000()
{
  static bool connected = false;

  if (wifi_port_is_connected(w3000, &connected, poll_for_new_w3000_connection, "w3000")) {
    while (w3000.available())
      p3000_decoder(w3000.read(), w3000_write);
    w3000.flush();
  }
}

static void poll_for_new_w2000_connection()
{
  static long next_bcast = 0;

  w2000 = W2000.available();
  if (w2000.connected()) {
    init_rxbuf(&w2000_rxbuf, "w2000_rx");
    /* Enable KEEPALIVE messages */
    int flags = 1;
    w2000.setSocketOption(SO_KEEPALIVE, (char *)&flags, sizeof(flags));  // boolean, turns on keepalives
    flags = 3000;
    w2000.setOption(TCP_KEEPALIVE, &flags);  // set keepalives to 3000msecs interval
    next_bcast = 0;
  } else {
    // SkyPortal app looks for these advertisement broadcasts:
    if (!next_bcast || time_after(millis(), next_bcast)) {
      udp.broadcastTo((uint8_t *)advertisement, strlen(advertisement), (uint16_t)55555);
      next_bcast = get_timeout(1000);
      if (verbose)
        Serial.println("UDP Broadcast");
    }
  }
}

static void service_w2000()
{
  static long w2000_timeout = 0;
  static bool connected = false;
  bool old_connected = connected;

  if (!wifi_port_is_connected(w2000, &connected, poll_for_new_w2000_connection, "w2000")) {
    w2000_timeout = 0;
  } else {
    while (w2000.available()) {
      (void)packet_decoder(&w2000_rxbuf, w2000.read());
      w2000_timeout = 0;
    }
    if (!w2000_timeout) {
      w2000_timeout = get_timeout(15000);  /* Same 15-sec timeout as Celestron WiFi */
    } else if (time_after(millis(), w2000_timeout)) {
      w2000_timeout = 0;
      w2000.stop();
      Serial.println("w2000 Timed-out");
    }
  }
  if (old_connected != connected)
    set_evo_wifi_off(connected);
}

#if ETH_ENABLED
#include "eth.h"
#endif

void loop()
{
  manage_evo_wifi_switch();  // Does nothing unless EVO_WIFI_OFF_PIN is defined
  manage_esp32_wifi_onoff();
  service_non_wifi_devices();
  service_w2000();
  service_w3000();
#if ETH_ENABLED
  ethernet_loop();
#endif
}

void setup()
{
  // Get the auxbus configured before _anything_ else:
  pinMode(AUXBUS_BUSYOUT_PIN,      OUTPUT);
  digitalWrite(AUXBUS_BUSYOUT_PIN, HIGH);  // Tri-state BUSYOUT
  pinMode(AUXBUS_BUSYIN_PIN,       INPUT);
  auxBus.begin(19200, SERIAL_8N2, AUXBUS_RX_PIN, AUXBUS_TX_PIN);
  auxBus.setRxBufferSize(900);  // Large enough for SS_CAMERA packets, plus some

#ifdef ESP32_RESTORE_DEFAULTS_PIN
  pinMode(ESP32_RESTORE_DEFAULTS_PIN, INPUT_PULLUP);
#endif
  pinMode(ESP32_WIFI_MODE_PIN, INPUT_PULLUP);
  pinMode(ESP32_WIFI_OFF_PIN,  INPUT_PULLUP);

  Serial.begin(115200, SERIAL_8N1);

  esp_read_mac(wifi_mac_addr, ESP_MAC_WIFI_STA);

  vars_restore();  // this also sets "verbose" appropriately
  init_rxbuf(&auxbus_rxbuf,    "auxbus_rx");
  init_rxbuf(&w2000_rxbuf,     "w2000_rx");

#if BT_ENABLED
  char bt_id[32];
  sprintf(bt_id, "%s-%02X%02X%02X", DEFAULT_SSID, wifi_mac_addr[3], wifi_mac_addr[4], wifi_mac_addr[5]);
  bt.begin(bt_id);
  init_rxbuf(&bt_rxbuf, "bt_rx");
#endif /* BT_ENABLED */

  gpsr.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  memset(auxbus_txq, 0, sizeof(auxbus_txq));
  Serial.println(ADAPTER_VERSION);
#if ETH_ENABLED
  ethernet_setup();
#endif
}
