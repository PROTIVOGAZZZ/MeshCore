#include "MyMesh.h"

#include "OfflineRoutePolicy.h"

#include <Arduino.h> // needed for PlatformIO
#include <ctype.h>
#ifdef ENABLE_WIFI_INTERFACE
#include <WiFi.h>
#endif
#include <Mesh.h>
#include <helpers/CompanionAppCapabilities.h>
#include <helpers/TextMessageAck.h>
#include <helpers/MCOCompatText.h>
#ifdef WITH_COMPANION_FRAME_FRAGMENTS
#include <helpers/CompanionFrameFragments.h>
#endif
#ifdef WITH_MCOTXT
#include <helpers/mcotxt/MCOtxtParts.h>
#include <helpers/mcotxt/MCOtxtTransport.h>
#endif
#ifdef WITH_MCMP_DETECT
#include <helpers/mcmp/MCMPDetect.h>
#endif
#ifdef WITH_AEIC_DETECT
#include <helpers/aeic/AEICDetect.h>
#endif
#ifdef WITH_MCOIMG_DETECT
#include <helpers/mcoimg_detect/MCOImgDetect.h>
#endif
#ifdef WITH_COMPANION_CLI
#include <helpers/CliReplySplitter.h>
#endif

static void normalizeCliCommand(char* buf);

#define CMD_APP_START                 1
#define CMD_SEND_TXT_MSG              2
#define CMD_SEND_CHANNEL_TXT_MSG      3
#define CMD_GET_CONTACTS              4 // with optional 'since' (for efficient sync)
#define CMD_GET_DEVICE_TIME           5
#define CMD_SET_DEVICE_TIME           6
#define CMD_SEND_SELF_ADVERT          7
#define CMD_SET_ADVERT_NAME           8
#define CMD_ADD_UPDATE_CONTACT        9
#define CMD_SYNC_NEXT_MESSAGE         10
#define SYNC_FLAG_FRAGMENT_ACK        0x01
#define CMD_SET_RADIO_PARAMS          11
#define CMD_SET_RADIO_TX_POWER        12
#define CMD_RESET_PATH                13
#define CMD_SET_ADVERT_LATLON         14
#define CMD_REMOVE_CONTACT            15
#define CMD_SHARE_CONTACT             16
#define CMD_EXPORT_CONTACT            17
#define CMD_IMPORT_CONTACT            18
#define CMD_REBOOT                    19
#define CMD_GET_BATT_AND_STORAGE      20   // was CMD_GET_BATTERY_VOLTAGE
#define CMD_SET_TUNING_PARAMS         21
#define CMD_DEVICE_QUERY              22
#define CMD_EXPORT_PRIVATE_KEY        23
#define CMD_IMPORT_PRIVATE_KEY        24
#define CMD_SEND_RAW_DATA             25
#define CMD_SEND_LOGIN                26
#define CMD_SEND_STATUS_REQ           27
#define CMD_HAS_CONNECTION            28
#define CMD_LOGOUT                    29 // 'Disconnect'
#define CMD_GET_CONTACT_BY_KEY        30
#define CMD_GET_CHANNEL               31
#define CMD_SET_CHANNEL               32
#define CMD_SIGN_START                33
#define CMD_SIGN_DATA                 34
#define CMD_SIGN_FINISH               35
#define CMD_SEND_TRACE_PATH           36
#define CMD_SET_DEVICE_PIN            37
#define CMD_SET_OTHER_PARAMS          38
#define CMD_SEND_TELEMETRY_REQ        39  // can deprecate this
#define CMD_GET_CUSTOM_VARS           40
#define CMD_SET_CUSTOM_VAR            41
#define CMD_GET_ADVERT_PATH           42
#define CMD_GET_TUNING_PARAMS         43
// NOTE: CMD range 44..49 parked, potentially for WiFi operations
#define CMD_SEND_BINARY_REQ           50
#define CMD_FACTORY_RESET             51
#define CMD_SEND_PATH_DISCOVERY_REQ   52
#define CMD_SET_FLOOD_SCOPE_KEY       54   // v8+
#define CMD_SEND_CONTROL_DATA         55   // v8+
#define CMD_GET_STATS                 56   // v8+, second byte is stats type
#define CMD_SEND_ANON_REQ             57
#define CMD_SET_AUTOADD_CONFIG        58
#define CMD_GET_AUTOADD_CONFIG        59
#define CMD_GET_ALLOWED_REPEAT_FREQ   60
#define CMD_SET_PATH_HASH_MODE        61
#define CMD_SEND_CHANNEL_DATA         62
#define CMD_SET_DEFAULT_FLOOD_SCOPE   63
#define CMD_GET_DEFAULT_FLOOD_SCOPE   64
#define CMD_SEND_RAW_PACKET           65
#define CMD_RUN_CLI_COMMAND           66  // v14+

// Stats sub-types for CMD_GET_STATS
#define STATS_TYPE_CORE               0
#define STATS_TYPE_RADIO              1
#define STATS_TYPE_PACKETS             2

#define RESP_CODE_OK                  0
#define RESP_CODE_ERR                 1
#define RESP_CODE_CONTACTS_START      2  // first reply to CMD_GET_CONTACTS
#define RESP_CODE_CONTACT             3  // multiple of these (after CMD_GET_CONTACTS)
#define RESP_CODE_END_OF_CONTACTS     4  // last reply to CMD_GET_CONTACTS
#define RESP_CODE_SELF_INFO           5  // reply to CMD_APP_START
#define RESP_CODE_SENT                6  // reply to CMD_SEND_TXT_MSG
#define RESP_CODE_CONTACT_MSG_RECV    7  // a reply to CMD_SYNC_NEXT_MESSAGE (ver < 3)
#define RESP_CODE_CHANNEL_MSG_RECV    8  // a reply to CMD_SYNC_NEXT_MESSAGE (ver < 3)
#define RESP_CODE_CURR_TIME           9  // a reply to CMD_GET_DEVICE_TIME
#define RESP_CODE_NO_MORE_MESSAGES    10 // a reply to CMD_SYNC_NEXT_MESSAGE
#define RESP_CODE_EXPORT_CONTACT      11
#define RESP_CODE_BATT_AND_STORAGE    12 // a reply to a CMD_GET_BATT_AND_STORAGE
#define RESP_CODE_DEVICE_INFO         13 // a reply to CMD_DEVICE_QUERY
#define RESP_CODE_PRIVATE_KEY         14 // a reply to CMD_EXPORT_PRIVATE_KEY
#define RESP_CODE_DISABLED            15
#define RESP_CODE_CONTACT_MSG_RECV_V3 16 // a reply to CMD_SYNC_NEXT_MESSAGE (ver >= 3)
#define RESP_CODE_CHANNEL_MSG_RECV_V3 17 // a reply to CMD_SYNC_NEXT_MESSAGE (ver >= 3)
#define RESP_CODE_CHANNEL_INFO        18 // a reply to CMD_GET_CHANNEL
#define RESP_CODE_SIGN_START          19
#define RESP_CODE_SIGNATURE           20
#define RESP_CODE_CUSTOM_VARS         21
#define RESP_CODE_ADVERT_PATH         22
#define RESP_CODE_TUNING_PARAMS       23
#define RESP_CODE_STATS               24   // v8+, second byte is stats type
#define RESP_CODE_AUTOADD_CONFIG      25
#define RESP_ALLOWED_REPEAT_FREQ      26
#define RESP_CODE_CHANNEL_DATA_RECV   27
#define RESP_CODE_DEFAULT_FLOOD_SCOPE 28
#define RESP_CODE_CLI_REPLY           29  // v14+, a reply to CMD_RUN_CLI_COMMAND

#define MAX_CHANNEL_DATA_LENGTH       (MAX_FRAME_SIZE - 9)

#define SEND_TIMEOUT_BASE_MILLIS        500
#define FLOOD_SEND_TIMEOUT_FACTOR       16.0f
#define DIRECT_SEND_PERHOP_FACTOR       6.0f
#define DIRECT_SEND_PERHOP_EXTRA_MILLIS 250
#define LAZY_CONTACTS_WRITE_DELAY       5000

// Remote CLI (see rcliCheckPin)
#define RCLI_MAX_BAD     5                 // wrong PINs in a row before a lock
#define RCLI_LOCK_MS     (10UL * 60 * 1000)
#define RCLI_MAX_SKEW    (15UL * 60)       // seconds, checked only when our clock is reliable
#define RCLI_SESSION_MS  (10UL * 60 * 1000) // CLI screen: commands without the PIN after one with it
#define LAZY_PREFS_WRITE_DELAY          2000
#define APP_DRAIN_GRACE_MILLIS          3000

#define FRAME_COMPAT_AEIC_NOTICE        0x01

#define PUBLIC_GROUP_PSK                "izOH6cXN6mrJ5e26oRXNcg=="


// these are _pushed_ to client app at any time
#define PUSH_CODE_ADVERT                0x80
#define PUSH_CODE_PATH_UPDATED          0x81
#define PUSH_CODE_SEND_CONFIRMED        0x82
#define PUSH_CODE_MSG_WAITING           0x83
#define PUSH_CODE_RAW_DATA              0x84
#define PUSH_CODE_LOGIN_SUCCESS         0x85
#define PUSH_CODE_LOGIN_FAIL            0x86
#define PUSH_CODE_STATUS_RESPONSE       0x87
#define PUSH_CODE_LOG_RX_DATA           0x88
#define PUSH_CODE_TRACE_DATA            0x89
#define PUSH_CODE_NEW_ADVERT            0x8A
#define PUSH_CODE_TELEMETRY_RESPONSE    0x8B
#define PUSH_CODE_BINARY_RESPONSE       0x8C
#define PUSH_CODE_PATH_DISCOVERY_RESPONSE 0x8D
#define PUSH_CODE_CONTROL_DATA          0x8E   // v8+
#define PUSH_CODE_CONTACT_DELETED       0x8F // used to notify client app of deleted contact when overwriting oldest
#define PUSH_CODE_CONTACTS_FULL         0x90 // used to notify client app that contacts storage is full

#define ERR_CODE_UNSUPPORTED_CMD        1
#define ERR_CODE_NOT_FOUND              2
#define ERR_CODE_TABLE_FULL             3
#define ERR_CODE_BAD_STATE              4
#define ERR_CODE_FILE_IO_ERROR          5
#define ERR_CODE_ILLEGAL_ARG            6

#define MAX_SIGN_DATA_LEN               (8 * 1024) // 8K

// Auto-add config bitmask
// Bit 0: If set, overwrite oldest non-favourite contact when contacts file is full
// Bits 1-4: these indicate which contact types to auto-add when manual_contact_mode = 0x01

namespace {

bool mapCyrToLatUtf8(uint8_t b0, uint8_t b1, char& out) {
  if (b0 == 0xD0) {
    switch (b1) {
      case 0x81: out = 'E'; return true; // Ё
      case 0x90: out = 'A'; return true; // А
      case 0x92: out = 'B'; return true; // В
      case 0x95: out = 'E'; return true; // Е
      case 0x97: out = '3'; return true; // З
      case 0x9A: out = 'K'; return true; // К
      case 0x9C: out = 'M'; return true; // М
      case 0x9D: out = 'H'; return true; // Н
      case 0x9E: out = 'O'; return true; // О
      case 0xA0: out = 'P'; return true; // Р
      case 0xA1: out = 'C'; return true; // С
      case 0xA2: out = 'T'; return true; // Т
      case 0xA5: out = 'X'; return true; // Х
      case 0xAC: out = 'b'; return true; // Ь
      case 0xB0: out = 'a'; return true; // а
      case 0xB5: out = 'e'; return true; // е
      case 0xBE: out = 'o'; return true; // о
      default: break;
    }
  } else if (b0 == 0xD1) {
    switch (b1) {
      case 0x80: out = 'p'; return true; // р
      case 0x81: out = 'c'; return true; // с
      case 0x83: out = 'y'; return true; // у
      case 0x85: out = 'x'; return true; // х
      case 0x91: out = 'e'; return true; // ё
      default: break;
    }
  }
  return false;
}

int transliterateChannelText(const char* src, int src_len, char* dest, int dest_size) {
  if (dest_size <= 0) return 0;

  int di = 0;
  for (int si = 0; si < src_len && di < dest_size - 1; ) {
    uint8_t b0 = (uint8_t)src[si];
    char mapped;
    if (si + 1 < src_len && mapCyrToLatUtf8(b0, (uint8_t)src[si + 1], mapped)) {
      dest[di++] = mapped;
      si += 2;
    } else {
      dest[di++] = src[si++];
    }
  }

  dest[di] = '\0';
  return di;
}

bool textDiffers(const char* a, int a_len, const char* b, int b_len) {
  return a_len != b_len || memcmp(a, b, a_len) != 0;
}

}
#define AUTO_ADD_OVERWRITE_OLDEST (1 << 0)  // 0x01 - overwrite oldest non-favourite when full
#define AUTO_ADD_CHAT             (1 << 1)  // 0x02 - auto-add Chat (Companion) (ADV_TYPE_CHAT)
#define AUTO_ADD_REPEATER         (1 << 2)  // 0x04 - auto-add Repeater (ADV_TYPE_REPEATER)
#define AUTO_ADD_ROOM_SERVER      (1 << 3)  // 0x08 - auto-add Room Server (ADV_TYPE_ROOM)
#define AUTO_ADD_SENSOR           (1 << 4)  // 0x10 - auto-add Sensor (ADV_TYPE_SENSOR)

static const uint32_t MIN_VALID_TS = 1577836800; // 2020-01-01 UTC
static const uint32_t MAX_VALID_TS = 2524608000; // 2050-01-01 UTC
static const uint32_t APP_TIME_HOLDOFF_SECS = 6 * 60 * 60;

void MyMesh::writeOKFrame() {
  uint8_t buf[1];
  buf[0] = RESP_CODE_OK;
  _serial->writeFrame(buf, 1);
}
void MyMesh::writeErrFrame(uint8_t err_code) {
  uint8_t buf[2];
  buf[0] = RESP_CODE_ERR;
  buf[1] = err_code;
  _serial->writeFrame(buf, 2);
}

void MyMesh::writeDisabledFrame() {
  uint8_t buf[1];
  buf[0] = RESP_CODE_DISABLED;
  _serial->writeFrame(buf, 1);
}

void MyMesh::writeContactRespFrame(uint8_t code, const ContactInfo &contact) {
  int i = 0;
  out_frame[i++] = code;
  memcpy(&out_frame[i], contact.id.pub_key, PUB_KEY_SIZE);
  i += PUB_KEY_SIZE;
  out_frame[i++] = contact.type;
  out_frame[i++] = contact.flags;
  out_frame[i++] = contact.out_path_len;
  memcpy(&out_frame[i], contact.out_path, MAX_PATH_SIZE);
  i += MAX_PATH_SIZE;
  StrHelper::strzcpy((char *)&out_frame[i], contact.name, 32);
  i += 32;
  memcpy(&out_frame[i], &contact.last_advert_timestamp, 4);
  i += 4;
  memcpy(&out_frame[i], &contact.gps_lat, 4);
  i += 4;
  memcpy(&out_frame[i], &contact.gps_lon, 4);
  i += 4;
  memcpy(&out_frame[i], &contact.lastmod, 4);
  i += 4;
  _serial->writeFrame(out_frame, i);
}

void MyMesh::updateContactFromFrame(ContactInfo &contact, uint32_t& last_mod, const uint8_t *frame, int len) {
  int i = 0;
  uint8_t code = frame[i++]; // eg. CMD_ADD_UPDATE_CONTACT
  memcpy(contact.id.pub_key, &frame[i], PUB_KEY_SIZE);
  i += PUB_KEY_SIZE;
  contact.type = frame[i++];
  contact.flags = frame[i++];
  contact.out_path_len = frame[i++];
  memcpy(contact.out_path, &frame[i], MAX_PATH_SIZE);
  i += MAX_PATH_SIZE;
  memcpy(contact.name, &frame[i], 32);
  i += 32;
  memcpy(&contact.last_advert_timestamp, &frame[i], 4);
  i += 4;
  if (len >= i + 8) { // optional fields
    memcpy(&contact.gps_lat, &frame[i], 4);
    i += 4;
    memcpy(&contact.gps_lon, &frame[i], 4);
    i += 4;
    if (len >= i + 4) {
      memcpy(&last_mod, &frame[i], 4);
    }
  }
}

// Frame storage: with fragments on a long frame lives in the shared pool; these
// helpers read through either representation so the queue code stays one path.
bool MyMesh::isQueuedChannelMsg(const Frame& frame) const {
  const uint8_t* data = frameData(frame);
  return data && (data[0] == RESP_CODE_CHANNEL_MSG_RECV ||
                  data[0] == RESP_CODE_CHANNEL_MSG_RECV_V3 ||
                  data[0] == RESP_CODE_CHANNEL_DATA_RECV);
}

int MyMesh::addToOfflineQueue(const uint8_t frame[], int len, uint8_t compat_flags, uint32_t seq) {
#ifdef WITH_COMPANION_FRAME_FRAGMENTS
  // The queue can hold FR01's full 259-byte original frame.
  if (!frame || len <= 0 || len > companion_frames::MAX_ORIGINAL_LEN) return -1;
#else
  // Frame::buf holds MAX_FRAME_SIZE bytes: a longer frame would overflow it and
  // a non-positive length makes no sense.
  if (!frame || len <= 0 || len > MAX_FRAME_SIZE) return -1;
#endif
  if (offline_queue_len >= OFFLINE_QUEUE_SIZE) {
    MESH_DEBUG_PRINTLN("WARN: offline_queue is full!");
    // A multi-part conversion keeps its original raw frame at queue[0] until
    // the final part is delivered. Never evict that active source midway.
#ifdef WITH_COMPANION_FRAME_FRAGMENTS
    int pos = (_app_sync_active || queued_fragment_id) ? 1 : 0;
#else
    int pos = _app_sync_active ? 1 : 0;
#endif
    while (pos < offline_queue_len && !isQueuedChannelMsg(offline_queue[pos])) pos++;
    if (pos >= offline_queue_len) {
      MESH_DEBUG_PRINTLN("INFO: no channel messages to remove from queue.");
      return -1;
    }
    removeOfflineQueueAt(pos);
    MESH_DEBUG_PRINTLN("INFO: removed oldest channel message from queue.");
  }

#ifdef WITH_COMPANION_FRAME_FRAGMENTS
  int slot = -1;
  if (len > MAX_FRAME_SIZE && _prefs.framed_packets_enabled) {
    slot = allocLongFrameSlot();
    if (slot < 0) {
      // The pool is full: give up the oldest long channel message instead.
      const int start = (_app_sync_active || queued_fragment_id) ? 1 : 0;
      for (int i = start; i < offline_queue_len; i++) {
        if (offline_queue[i].len > MAX_FRAME_SIZE && isQueuedChannelMsg(offline_queue[i])) {
          removeOfflineQueueAt(i);
          slot = allocLongFrameSlot();
          break;
        }
      }
    }
  }
#endif

  const int pos = offline_queue_len;
  Frame& queued = offline_queue[pos];
  queued.len = len;
#ifdef WITH_COMPANION_FRAME_FRAGMENTS
  if (len <= MAX_FRAME_SIZE) {
    memcpy(queued.storage.inline_buf, frame, len);
  } else if (slot >= 0) {
    queued.storage.long_slot = (uint8_t)slot;
    memcpy(long_frame_pool[slot], frame, len);
  } else {
    const int legacy_len = buildLegacyQueuedFrame(frame, len, queued.storage.inline_buf);
    if (!legacy_len) {
      queued.len = 0;
      MESH_DEBUG_PRINTLN("long-frame pool full: unsafe frame type %02X dropped", frame[0]);
      return -1;
    }
    queued.len = (uint16_t)legacy_len;
    MESH_DEBUG_PRINTLN("long-frame pool full: stored legacy frame");
  }
#else
  memcpy(queued.buf, frame, len);
#endif
  initQueueEntry(pos, compat_flags, seq);
  offline_queue_len++;
  return pos;
}

void MyMesh::removeOfflineQueueAt(int pos) {
  if (pos < 0 || pos >= offline_queue_len) return;
  onQueueEntryRemoved(pos);
#ifdef WITH_COMPANION_FRAME_FRAGMENTS
  releaseFrameStorage(offline_queue[pos]);
#endif
  for (int i = pos; i < offline_queue_len - 1; i++) {
    offline_queue[i] = offline_queue[i + 1];
    moveQueueAux(i, i + 1);
  }
  offline_queue_len--;
  clearQueueAux(offline_queue_len);
}

void MyMesh::removeOfflineQueueHead() {
  if (offline_queue_len <= 0) return;
  // Nothing to count here: the store's watermark is the oldest entry still
  // queued, taken from the queue itself by serviceOfflineStore().
  removeOfflineQueueAt(0);
}

// The per-entry bookkeeping that travels with a queue entry when the queue shifts.
void MyMesh::moveQueueAux(int dst, int src) {
  offline_queue_store_pending[dst] = offline_queue_store_pending[src];
  offline_queue_seq[dst] = offline_queue_seq[src];
  offline_queue_route[dst] = offline_queue_route[src];
#if WITH_OFFLINE_ROUTES
  offline_queue_live[dst] = offline_queue_live[src];
#endif
}

void MyMesh::clearQueueAux(int pos) {
  offline_queue[pos].compat_flags = 0;
  offline_queue_store_pending[pos] = 0;
  offline_queue_seq[pos] = offline_ring::NO_SEQ;
  offline_queue_route[pos] = OFFLINE_ROUTE_NONE;
#if WITH_OFFLINE_ROUTES
  offline_queue_live[pos] = OFFLINE_LIVE_NONE;
#endif
}

// A new entry at `pos`. A frame coming back from the offline store keeps its
// sequence number; a new one takes the next and waits for loop() to write it.
void MyMesh::initQueueEntry(int pos, uint8_t compat_flags, uint32_t seq) {
  offline_queue[pos].compat_flags = compat_flags;
  offline_queue_route[pos] = OFFLINE_ROUTE_NONE;
#if WITH_OFFLINE_ROUTES
  offline_queue_live[pos] = OFFLINE_LIVE_NONE;
#endif
  if (seq != offline_ring::NO_SEQ) {
    offline_queue_seq[pos] = seq;
    offline_queue_store_pending[pos] = 0;   // it is in the store already
    return;
  }
  offline_queue_seq[pos] = offline_store.takeSeq();
  offline_queue_store_pending[pos] = 1;
  const unsigned long now = millis() | 1;   // 0 means "nothing pending"
  offline_last_enqueue = now;
  if (offline_pending_since == 0) offline_pending_since = now;
}

void MyMesh::onQueueEntryRemoved(int pos) {
#if WITH_OFFLINE_ROUTES
  if (offline_queue_route[pos] == OFFLINE_ROUTE_POOL) {
    const int i = findPoolRoute(offline_queue_seq[pos]);
    if (i >= 0) route_pool[i].seq = offline_ring::NO_SEQ;
  }
#else
  (void)pos;
#endif
}

void MyMesh::resetAppSyncState() {
  _app_sync_active = false;
  _app_sync_original_sent = false;
  _app_sync_route_sent = false;
  _app_sync_next_part = 0;
  _app_sync_timestamp = 0;
}

#ifdef WITH_COMPANION_FRAME_FRAGMENTS

bool MyMesh::canSendFrameFragments() const {
  return _prefs.framed_packets_enabled && _app_supports_frmfrg1;
}

void MyMesh::resetFragmentTransferState() {
  queued_fragment_id = 0;
  queued_fragment_index = 0;
  queued_fragment_ack_pending = false;
}

void MyMesh::resetFragmentSession() {
  resetFragmentTransferState();
  resetAppSyncState();
  _app_supports_frmfrg1 = false;
  _app_supports_mctxt = false;
  _app_supports_mcmp = false;
  _app_supports_aeic = false;
  _app_supports_mcoimg = false;
}

uint16_t MyMesh::allocFragmentId() {
  if (!next_fragment_id) next_fragment_id = 1;
  return next_fragment_id++;
}

void MyMesh::writeFrameMaybeFragmented(const uint8_t frame[], uint16_t len, uint8_t flags) {
  if (!_serial || !_serial->isConnected()) return;
  if (len <= MAX_FRAME_SIZE) {
    _serial->writeFrame(frame, len);
    return;
  }
  if (!canSendFrameFragments() || len > companion_frames::MAX_ORIGINAL_LEN) {
    // This is exactly the case where a long message arrives but its route never
    // does: a raw feed record can exceed MAX_FRAME_SIZE, so it is dropped here
    // unless the app declared `frmfrg1` and framed packets are enabled.
    OQ_LOG("frame dropped: len=%u frag=%d pref=%d max=%u\n",
           (unsigned)len, canSendFrameFragments() ? 1 : 0,
           _prefs.framed_packets_enabled ? 1 : 0,
           (unsigned)companion_frames::MAX_ORIGINAL_LEN);
    MESH_DEBUG_PRINTLN("live companion frame too long: %u", len);
    return;
  }

  const uint16_t id = allocFragmentId();
  const uint8_t count = companion_frames::fragmentCount(len);
  for (uint8_t index = 0; index < count; index++) {
    uint8_t fragment[MAX_FRAME_SIZE];
    const size_t n = companion_frames::buildFragment(fragment, sizeof(fragment), frame, len, id, index, flags);
    if (n) _serial->writeFrame(fragment, n);
  }
}

void MyMesh::writePacketPayloadFrameMaybeFragmented(mesh::Packet* packet, uint8_t push_code, uint8_t path_byte, const char* name) {
  if (!_serial || !_serial->isConnected()) {
    MESH_DEBUG_PRINTLN("%s: app offline", name);
    return;
  }
  const uint16_t len = packet->payload_len + 4;
  if (len > companion_frames::MAX_ORIGINAL_LEN) {
    MESH_DEBUG_PRINTLN("%s: payload too long", name);
    return;
  }

  uint8_t frame[companion_frames::MAX_ORIGINAL_LEN];
  frame[0] = push_code;
  frame[1] = (int8_t)(_radio->getLastSNR() * 4);
  frame[2] = (int8_t)_radio->getLastRSSI();
  frame[3] = path_byte;
  memcpy(&frame[4], packet->payload, packet->payload_len);
  writeFrameMaybeFragmented(frame, len);
}

int MyMesh::allocLongFrameSlot() {
  for (int i = 0; i < OFFLINE_LONG_FRAME_SLOTS; i++) {
    const uint8_t bit = 1u << i;
    if (!(long_frame_slot_mask & bit)) {
      long_frame_slot_mask |= bit;
      return i;
    }
  }
  return -1;
}

void MyMesh::freeLongFrameSlot(uint8_t slot) {
  if (slot >= OFFLINE_LONG_FRAME_SLOTS) {
    MESH_DEBUG_PRINTLN("invalid long-frame slot: %u", slot);
    return;
  }
  const uint8_t bit = 1u << slot;
  if (!(long_frame_slot_mask & bit)) {
    MESH_DEBUG_PRINTLN("long-frame slot already free: %u", slot);
    return;
  }
  long_frame_slot_mask &= ~bit;
}

uint8_t* MyMesh::frameData(Frame& frame) {
  if (frame.len <= MAX_FRAME_SIZE) return frame.storage.inline_buf;
  const uint8_t slot = frame.storage.long_slot;
  if (slot >= OFFLINE_LONG_FRAME_SLOTS || !(long_frame_slot_mask & (1u << slot))) return nullptr;
  return long_frame_pool[slot];
}

const uint8_t* MyMesh::frameData(const Frame& frame) const {
  if (frame.len <= MAX_FRAME_SIZE) return frame.storage.inline_buf;
  const uint8_t slot = frame.storage.long_slot;
  if (slot >= OFFLINE_LONG_FRAME_SLOTS || !(long_frame_slot_mask & (1u << slot))) return nullptr;
  return long_frame_pool[slot];
}

void MyMesh::releaseFrameStorage(Frame& frame) {
  if (frame.len > MAX_FRAME_SIZE) freeLongFrameSlot(frame.storage.long_slot);
  frame.len = 0;
}

// A long frame whose RAM slot could not be taken is stored collapsed to what a
// legacy client can take. Returns the stored length, or 0 when the type cannot
// be truncated safely.
int MyMesh::buildLegacyQueuedFrame(const uint8_t* src, uint16_t len, uint8_t* dest) const {
  if (!src || !dest || !len) return 0;
  const uint16_t copy_len = len > MAX_FRAME_SIZE ? MAX_FRAME_SIZE : len;
  switch (src[0]) {
    case RESP_CODE_CONTACT_MSG_RECV:
    case RESP_CODE_CONTACT_MSG_RECV_V3:
    case RESP_CODE_CHANNEL_MSG_RECV:
    case RESP_CODE_CHANNEL_MSG_RECV_V3:
      memcpy(dest, src, copy_len);
      return copy_len;
    case RESP_CODE_CHANNEL_DATA_RECV:
      if (copy_len < 9) return 0;
      memcpy(dest, src, copy_len);
      dest[8] = copy_len - 9;
      return copy_len;
    default:
      if (len > MAX_FRAME_SIZE) return 0;
      memcpy(dest, src, copy_len);
      return copy_len;
  }
}

// One app-facing representation of the queue head's original frame. When
// `retain_source` is true the entry stays in the queue for a following derived
// compatibility text; otherwise it leaves once the original is fully handed
// over. A long frame is delivered as ACKed FR01 fragments. Returns the bytes to
// send (0 when there is nothing to send this call); `stage_complete` says the
// original has been delivered in full.
int MyMesh::emitQueuedOriginalForApp(uint8_t frame[], bool retain_source,
                                     bool has_ack, uint16_t ack_id,
                                     uint8_t ack_index, bool& stage_complete) {
  stage_complete = false;
  if (offline_queue_len <= 0) return 0;
  Frame& source = offline_queue[0];
  const uint8_t* data = frameData(source);
  if (!data) {
    MESH_DEBUG_PRINTLN("corrupt offline long-frame slot: entry dropped");
    removeOfflineQueueHead();
    resetFragmentTransferState();
    resetAppSyncState();
    stage_complete = true;
    return 0;
  }

  if (source.len <= MAX_FRAME_SIZE) {
    const int result_len = source.len;
    memcpy(frame, data, result_len);
    if (retain_source) {
      _app_sync_original_sent = true;
    } else {
      removeOfflineQueueHead();
      resetAppSyncState();
    }
    resetFragmentTransferState();
    stage_complete = true;
    return result_len;
  }

  if (!canSendFrameFragments()) {
    // The node setting is off or the app did not declare `frmfrg1`: collapse the
    // long frame to what a legacy client can take.
    const uint8_t original_type = data[0];
    const int result_len = buildLegacyQueuedFrame(data, source.len, frame);
    if (retain_source) {
      _app_sync_original_sent = true;
    } else {
      removeOfflineQueueHead();
      resetAppSyncState();
    }
    resetFragmentTransferState();
    stage_complete = true;
    if (!result_len) {
      MESH_DEBUG_PRINTLN("unsafe long frame type %02X cannot be delivered to legacy app",
                         original_type);
    }
    return result_len;
  }

  const uint8_t count = companion_frames::fragmentCount(source.len);
  if (!queued_fragment_id) {
    queued_fragment_id = allocFragmentId();
    queued_fragment_index = 0;
    queued_fragment_ack_pending = false;
  }
  if (queued_fragment_ack_pending && has_ack &&
      ack_id == queued_fragment_id && ack_index == queued_fragment_index) {
    queued_fragment_ack_pending = false;
    if (++queued_fragment_index >= count) {
      if (retain_source) {
        _app_sync_original_sent = true;
      } else {
        removeOfflineQueueHead();
        resetAppSyncState();
      }
      resetFragmentTransferState();
      stage_complete = true;
      return 0;
    }
  }

  const size_t result_len = companion_frames::buildFragment(
      frame, MAX_FRAME_SIZE, data, source.len, queued_fragment_id,
      queued_fragment_index, companion_frames::FLAG_QUEUED);
  queued_fragment_ack_pending = result_len > 0;
  return (int)result_len;
}

#endif  // WITH_COMPANION_FRAME_FRAGMENTS

// Canonical queue frames are stored in the v3 layout (SNR + two reserved bytes).
// A legacy app (< v3) gets the collapse here: rewrite the code (16->7, 17->8)
// and drop the three bytes after it. Returns the new length, or 0 when the
// frame is not a v3 text-message frame (caller keeps it as-is).
int MyMesh::convertV3FrameToLegacy(const uint8_t* src, int len, uint8_t* dst) const {
  if (!src || !dst || len < 4) return 0;
  const uint8_t code = src[0];
  if (code != RESP_CODE_CONTACT_MSG_RECV_V3 && code != RESP_CODE_CHANNEL_MSG_RECV_V3) return 0;
  dst[0] = (code == RESP_CODE_CONTACT_MSG_RECV_V3) ? RESP_CODE_CONTACT_MSG_RECV
                                                   : RESP_CODE_CHANNEL_MSG_RECV;
  memmove(dst + 1, src + 4, (size_t)(len - 4));
  return len - 3;
}

float MyMesh::getAirtimeBudgetFactor() const {
  return _prefs.airtime_factor;
}

int MyMesh::getInterferenceThreshold() const {
  return _prefs.interference_threshold;
}
bool MyMesh::getCADEnabled() const {
  return _prefs.cad_enabled;
}

int MyMesh::calcRxDelay(float score, uint32_t air_time) const {
  if (_prefs.rx_delay_base <= 0.0f) return 0;
  return (int)((pow(_prefs.rx_delay_base, 0.85f - score) - 1.0) * air_time);
}

uint32_t MyMesh::getRetransmitDelay(const mesh::Packet *packet) {
  uint32_t t = (_radio->getEstAirtimeFor(packet->getPathByteLen() + packet->payload_len + 2) * _prefs.tx_delay_factor);
  return getRNG()->nextInt(0, 5*t + 1);
}
uint32_t MyMesh::getDirectRetransmitDelay(const mesh::Packet *packet) {
  uint32_t t = (_radio->getEstAirtimeFor(packet->getPathByteLen() + packet->payload_len + 2) * _prefs.direct_tx_delay_factor);
  return getRNG()->nextInt(0, 5*t + 1);
}

uint8_t MyMesh::getExtraAckTransmitCount() const {
  return _prefs.multi_acks;
}

void MyMesh::logRxRaw(float snr, float rssi, const uint8_t raw[], int len) {
  const uint8_t* log_raw = raw;
  int log_len = len;
  uint8_t mapped[MAX_TRANS_UNIT];
  int mapped_len = mapCyr2LatChannelRawLog(raw, len, mapped, sizeof(mapped));
  if (mapped_len > 0) {
    log_raw = mapped;
    log_len = mapped_len;
  }
  // No app attached: nothing to push. The route of a message heard meanwhile is
  // kept with the queued message itself (keepRoute), not here: only the copy that
  // produced a message matters, and this runs for every packet on the air.
  if (!_serial || !_serial->isConnected()) return;
#ifdef WITH_COMPANION_FRAME_FRAGMENTS
  if (log_len + 3 <= companion_frames::MAX_ORIGINAL_LEN) {
    uint8_t frame[companion_frames::MAX_ORIGINAL_LEN];
    int i = 0;
    frame[i++] = PUSH_CODE_LOG_RX_DATA;
    frame[i++] = (int8_t)(snr * 4);
    frame[i++] = (int8_t)(rssi);
    memcpy(&frame[i], log_raw, log_len);
    i += log_len;
    writeFrameMaybeFragmented(frame, i);
  }
#else
  if (log_len + 3 <= MAX_FRAME_SIZE) {
    int i = 0;
    out_frame[i++] = PUSH_CODE_LOG_RX_DATA;
    out_frame[i++] = (int8_t)(snr * 4);
    out_frame[i++] = (int8_t)(rssi);
    memcpy(&out_frame[i], log_raw, log_len);
    i += log_len;

    _serial->writeFrame(out_frame, i);
  }
#endif
}

// The offline store lives in a filesystem: one small file per record, plus one
// file holding the ring index. Handles are opened for a single call and closed
// again, so nothing dangles across a disconnect or a reset.

static File offlineOpenRead(FILESYSTEM* fs, const char* path) {
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  return fs->open(path, FILE_O_READ);
#elif defined(RP2040_PLATFORM)
  return fs->open(path, "r");
#else
  return fs->open(path, "r", false);
#endif
}

static File offlineOpenWrite(FILESYSTEM* fs, const char* path) {
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  return fs->open(path, FILE_O_WRITE);
#elif defined(RP2040_PLATFORM)
  return fs->open(path, "w");
#else
  return fs->open(path, "w", true);
#endif
}

// The records go to the filesystem that has room for them. On the nRF52 boards
// that is the second one: the primary filesystem (InternalFS on the T114) is a
// 28 KB region shared with the identity and the BLE bonds, while the contacts and
// channels filesystem is a 100 KB region of its own. The space check asks the
// same filesystem it writes to.
static FILESYSTEM* offlineStoreFS(DataStore* store) {
  FILESYSTEM* fs = store->getSecondaryFS();
  if (fs) return fs;
  return store->getPrimaryFS();
}

#define OFFLINE_STORE_RECORD_FMT "/oq%02x.bin"
#define OFFLINE_STORE_INDEX "/oqidx.bin"

static void offlineRecordPath(char* path, size_t size, uint16_t slot) {
  snprintf(path, size, OFFLINE_STORE_RECORD_FMT, (unsigned)slot);
}

// Earlier layouts kept a separate raw feed ring (/rf*.bin, /rfidx.bin, before that
// /rawfeed.bin) and a queue ring (/rq*.bin, /rqidx.bin). Nothing reads them any
// more, and they hold space the contacts need. Their indexes are what say the
// record files are there, so the sweep runs only while one is still present.
static void offlineDropLegacy(FILESYSTEM* fs) {
  static const char* const singles[] = { "/rawfeed.bin", "/rfidx.bin", "/rqidx.bin" };
  bool any = false;
  for (const char* path : singles) {
    if (fs->exists(path)) {
      fs->remove(path);
      any = true;
    }
  }
  if (!any) return;
  char path[16];
  for (uint16_t s = 0; s < 64; s++) {
    snprintf(path, sizeof(path), "/rf%02x.bin", (unsigned)s);
    if (fs->exists(path)) fs->remove(path);
    snprintf(path, sizeof(path), "/rq%02x.bin", (unsigned)s);
    if (fs->exists(path)) fs->remove(path);
  }
  OQ_LOG("store: removed the earlier layout\n");
}

// A record file is replaced, never rewritten: on the nRF52 core FILE_O_WRITE opens
// a file and seeks to its end, so a write "at an offset" lands at the end and
// still reports success. The ring reads every write back, because LittleFS
// commits a file on close() and close() reports nothing through this API.
static bool offlineWriteFile(FILESYSTEM* fs, const char* path, const uint8_t* data, size_t len) {
  if (!fs) return false;
  if (fs->exists(path)) fs->remove(path);
  File f = offlineOpenWrite(fs, path);
  if (!f) return false;
  const bool ok = f.write(data, len) == len;
  f.close();
  return ok;
}

static int offlineReadFile(FILESYSTEM* fs, const char* path, uint8_t* buf, size_t cap) {
  if (!fs) return -1;
  File f = offlineOpenRead(fs, path);
  if (!f) return -1;
  const int n = (int)f.read(buf, cap);
  f.close();
  return n;
}

bool MyMesh::OfflineFsStorage::writeSlot(uint16_t slot, const uint8_t* data, size_t len) {
  char path[16];
  offlineRecordPath(path, sizeof(path), slot);
  return offlineWriteFile(offlineStoreFS(store), path, data, len);
}

int MyMesh::OfflineFsStorage::readSlot(uint16_t slot, uint8_t* buf, size_t cap) {
  char path[16];
  offlineRecordPath(path, sizeof(path), slot);
  return offlineReadFile(offlineStoreFS(store), path, buf, cap);
}

void MyMesh::OfflineFsStorage::removeSlot(uint16_t slot) {
  FILESYSTEM* fs = offlineStoreFS(store);
  if (!fs) return;
  char path[16];
  offlineRecordPath(path, sizeof(path), slot);
  if (fs->exists(path)) fs->remove(path);
}

bool MyMesh::OfflineFsStorage::writeIndex(const uint8_t* data, size_t len) {
  return offlineWriteFile(offlineStoreFS(store), OFFLINE_STORE_INDEX, data, len);
}

int MyMesh::OfflineFsStorage::readIndex(uint8_t* buf, size_t cap) {
  return offlineReadFile(offlineStoreFS(store), OFFLINE_STORE_INDEX, buf, cap);
}

void MyMesh::beginOfflineStore() {
  FILESYSTEM* fs = offlineStoreFS(_store);
  if (!fs) return;
  offlineDropLegacy(fs);
  FILESYSTEM* primary = _store->getPrimaryFS();
  if (primary && primary != fs) offlineDropLegacy(primary);

#if OFFLINE_STORE_OFF
  OQ_LOG("store: off by build flag\n");
  return;
#endif

  if (!offline_store.open()) {
    // No usable index: start over. The store shares the filesystem with the
    // contacts, so it only takes space when there is clearly enough for both:
    // every slot at its largest, plus a share of the filesystem (never under
    // 16 KB) left for the contacts to keep growing.
    const uint32_t total_kb = _store->getStorageTotalKb();
    const uint32_t used_kb = _store->getStorageUsedKb();
    const uint32_t need_kb =
        ((uint32_t)OFFLINE_STORE_SLOTS * (sizeof(offline_scratch) + 64) + 1023) / 1024 + 1;
    uint32_t reserve_kb = total_kb / 6;
    if (reserve_kb < 16) reserve_kb = 16;
    if (used_kb + need_kb + reserve_kb > total_kb) {
      OQ_LOG("store: off - %lu KB total, %lu used, %lu needed, %lu reserved\n",
             (unsigned long)total_kb, (unsigned long)used_kb,
             (unsigned long)need_kb, (unsigned long)reserve_kb);
      return;
    }
    if (!offline_store.create()) {
      OQ_LOG("store: cannot create the index, off\n");
      return;
    }
    OQ_LOG("store: created, %u slots, %lu KB needed of %lu KB free\n", (unsigned)OFFLINE_STORE_SLOTS,
           (unsigned long)need_kb, (unsigned long)(total_kb - used_kb));
    return;
  }

  // Hand what the app has not taken yet back to the queue, oldest first, each
  // frame under its own sequence number. The routes stay in the store and are
  // read again when their message goes out.
  uint32_t seqs[OFFLINE_STORE_SLOTS];
  const int n = offline_store.pending(seqs, OFFLINE_STORE_SLOTS);
  const uint32_t now = getRTCClock()->getCurrentTime();
  // A frame is only too old if the clock that says so could be right: a volatile
  // clock starts at a fixed date and would age every record it sees.
  const bool clock_set = (_time_source != TIME_SOURCE_UNSET);
  uint8_t frame[OFFLINE_STORE_MAX_FRAME];
  uint16_t restored = 0, lost = 0, stale = 0;
  for (int k = 0; k < n; k++) {
    offline_ring::Record r;
    if (!offline_store.get(seqs[k], r, frame, nullptr)) {
      lost++;   // the slot no longer reads back whole
      continue;
    }
    if (r.at_s != 0 && clock_set && now > r.at_s && now - r.at_s > OFFLINE_STORE_TTL_SECONDS) {
      stale++;
      continue;
    }
    const int pos = addToOfflineQueue(frame, r.frame_len, r.flags, r.seq);
    if (pos < 0) {
      lost++;
      continue;
    }
#if WITH_OFFLINE_ROUTES
    if (r.route_len > 0) offline_queue_route[pos] = OFFLINE_ROUTE_STORED;
#endif
    restored++;
  }
  offline_last_restored = restored;
  offline_last_lost = lost;
  OQ_LOG("store: opened, %d held, %u restored, %u stale, %u lost (next=%lu acked=%lu)\n", n,
         (unsigned)restored, (unsigned)stale, (unsigned)lost,
         (unsigned long)offline_store.nextSeq(), (unsigned long)offline_store.acked());
}

// Called from loop(), and only while no BLE link is up. Writes the entries the
// app has not taken once the queue has been quiet for a moment, then moves the
// watermark up to the oldest entry still queued.
void MyMesh::serviceOfflineStore() {
  if (!offline_store.usable()) return;
  const unsigned long now = millis();
  if (offline_pending_since != 0 &&
      (now - offline_last_enqueue >= OFFLINE_STORE_QUIET_MS ||
       now - offline_pending_since >= OFFLINE_STORE_MAX_WAIT_MS)) {
    uint8_t budget = OFFLINE_STORE_FLUSH_BATCH;
    bool more = false;
    for (int pos = 0; pos < offline_queue_len; pos++) {
      if (!offline_queue_store_pending[pos]) continue;
      if (budget == 0) {
        more = true;
        break;
      }
      budget--;
      const uint8_t* data = frameData(offline_queue[pos]);
      const uint16_t len = (uint16_t)offline_queue[pos].len;
      if (!data || len == 0 || len > OFFLINE_STORE_MAX_FRAME) {
        offline_queue_store_pending[pos] = 0;
        continue;
      }
      offline_ring::Record r;
      r.seq = offline_queue_seq[pos];
      r.at_s = getRTCClock()->getCurrentTime();
      r.flags = offline_queue[pos].compat_flags;
      r.frame_len = len;
      r.route_len = 0;
      const uint8_t* route = nullptr;
#if WITH_OFFLINE_ROUTES
      const int pool = offline_queue_route[pos] == OFFLINE_ROUTE_POOL ? findPoolRoute(r.seq) : -1;
      if (pool >= 0) {
        route = route_pool[pool].data;
        r.route_len = route_pool[pool].len;
      }
#endif
      if (!offline_store.put(r, data, route)) {
        OQ_LOG("store: write failed seq=%lu (store %s)\n", (unsigned long)r.seq,
               offline_store.usable() ? "on" : "now off");
        more = true;   // another chance next pass, unless the store gave up
        break;
      }
      offline_queue_store_pending[pos] = 0;
#if WITH_OFFLINE_ROUTES
      if (pool >= 0) {
        // The route now lives in the record; the RAM copy is free for the next one.
        offline_queue_route[pos] = OFFLINE_ROUTE_STORED;
        route_pool[pool].seq = offline_ring::NO_SEQ;
      }
#endif
      OQ_LOG("store: seq=%lu len=%u route=%u written\n", (unsigned long)r.seq,
             (unsigned)len, (unsigned)r.route_len);
    }
    if (!more) offline_pending_since = 0;
  }
  if (!offline_store.usable()) return;
  // Everything below the oldest entry still queued has been taken by the app or
  // dropped on purpose; either way it is not to come back after a reboot.
  offline_store.setAcked(offline_queue_len > 0 ? offline_queue_seq[0] : offline_store.nextSeq());
  if (offline_store.indexDirty() && now - offline_last_enqueue >= OFFLINE_STORE_QUIET_MS) {
    offline_store.flushIndex();
  }
}

#if WITH_OFFLINE_ROUTES
int MyMesh::findPoolRoute(uint32_t seq) const {
  for (int i = 0; i < OFFLINE_ROUTE_POOL_SLOTS; i++) {
    if (route_pool[i].seq == seq) return i;
  }
  return -1;
}

// Keep the route of a message queued at `pos`: the raw packet of the copy this
// node decoded, path as received. It is the copy whose hop count the queued
// frame reports. Only flood-routed channel messages carry a path worth showing.
void MyMesh::keepRoute(int pos, const mesh::Packet* pkt) {
  if (pos < 0 || pos >= offline_queue_len || !pkt || !pkt->isRouteFlood()) return;
  const uint8_t type = pkt->getPayloadType();
  if (!offline_route::keep(type)) return;

  uint8_t raw[MAX_TRANS_UNIT + 1];
  int len = pkt->writeTo(raw);
  // The same mapping as the live push, so the app decodes the text the queued
  // frame carries (it matches a channel copy by its decrypted content).
  uint8_t mapped[MAX_TRANS_UNIT + 1];
  const int mapped_len = mapCyr2LatChannelRawLog(raw, len, mapped, sizeof(mapped));
  const uint8_t* src = raw;
  if (mapped_len > 0) {
    src = mapped;
    len = mapped_len;
  }
  if (len <= 0 || len + 2 > OFFLINE_ROUTE_MAX) return;

  // A free slot, or else the one holding the oldest route: the newest messages
  // keep theirs. loop() empties the pool within seconds while no app is attached,
  // so this only bites when the store cannot take records at all.
  int slot = -1;
  for (int i = 0; i < OFFLINE_ROUTE_POOL_SLOTS; i++) {
    if (route_pool[i].seq == offline_ring::NO_SEQ) {
      slot = i;
      break;
    }
    if (slot < 0 || route_pool[i].seq < route_pool[slot].seq) slot = i;
  }
  if (route_pool[slot].seq != offline_ring::NO_SEQ) {
    for (int q = 0; q < offline_queue_len; q++) {
      if (offline_queue_seq[q] == route_pool[slot].seq) {
        offline_queue_route[q] = OFFLINE_ROUTE_NONE;
        break;
      }
    }
    OQ_LOG("route: pool full, seq=%lu loses its route\n", (unsigned long)route_pool[slot].seq);
  }
  RoutePoolEntry& e = route_pool[slot];
  e.seq = offline_queue_seq[pos];
  e.len = (uint16_t)(len + 2);
  e.data[0] = (uint8_t)pkt->_snr;    // SNR x4, as in a LogRxData push
  e.data[1] = (uint8_t)pkt->_rssi;
  memcpy(&e.data[2], src, (size_t)len);
  offline_queue_route[pos] = OFFLINE_ROUTE_POOL;
  // An attached app got this very copy live, as a LogRxData push. The route is
  // kept anyway: the app may disconnect before it takes the message, and the next
  // session — or another app — needs it. The session number keeps the current one
  // from being handed the same copy twice, which it would count as a relay. On a
  // wired link isConnected() is always true, so a session only starts when an app
  // actually says CMD_APP_START.
  offline_queue_live[pos] = (_serial && _serial->isConnected()) ? app_session : OFFLINE_LIVE_NONE;
  OQ_LOG("route: kept seq=%lu len=%u type=%u route=%u hops=%u live=%u\n", (unsigned long)e.seq,
         (unsigned)e.len, (unsigned)type, (unsigned)(pkt->header & PH_ROUTE_MASK),
         (unsigned)pkt->getPathHashCount(), (unsigned)offline_queue_live[pos]);
}

// Push the route of the queue entry at `pos` as a LogRxData frame, right before
// the entry itself goes out.
void MyMesh::emitQueuedRoute(int pos) {
  if (pos < 0 || pos >= offline_queue_len) return;
  if (!_serial || !_serial->isConnected()) return;
  if (offline_queue_live[pos] == app_session) return;   // this session saw it live
  uint8_t frame[1 + OFFLINE_ROUTE_MAX];
  uint16_t len = 0;
  if (offline_queue_route[pos] == OFFLINE_ROUTE_POOL) {
    const int i = findPoolRoute(offline_queue_seq[pos]);
    if (i >= 0) {
      len = route_pool[i].len;
      memcpy(&frame[1], route_pool[i].data, len);
    }
  } else if (offline_queue_route[pos] == OFFLINE_ROUTE_STORED) {
    offline_ring::Record r;
    if (offline_store.get(offline_queue_seq[pos], r, nullptr, &frame[1])) len = r.route_len;
  }
  if (len < 3) return;   // snr, rssi and at least a packet header
  const uint8_t hdr = frame[3];   // raw packet header, after snr and rssi
  const uint8_t type = (uint8_t)((hdr >> PH_TYPE_SHIFT) & PH_TYPE_MASK);
  if (!offline_route::deliver(type, _app_info, _app_info_len)) return;
  frame[0] = PUSH_CODE_LOG_RX_DATA;
#ifdef WITH_COMPANION_FRAME_FRAGMENTS
  writeFrameMaybeFragmented(frame, (uint16_t)(len + 1));
#else
  if (len + 1 <= MAX_FRAME_SIZE) {
    _serial->writeFrame(frame, len + 1);
  } else {
    // Without fragments a raw packet past MAX_FRAME_SIZE cannot reach the app.
    OQ_LOG("route: seq=%lu too long (%u) without fragments\n",
           (unsigned long)offline_queue_seq[pos], (unsigned)(len + 1));
    return;
  }
#endif
  OQ_LOG("route: seq=%lu sent len=%u\n", (unsigned long)offline_queue_seq[pos], (unsigned)len);
}
#endif

bool MyMesh::isAutoAddEnabled() const {
  return (_prefs.manual_add_contacts & 1) == 0;
}

bool MyMesh::shouldAutoAddContactType(uint8_t contact_type) const {
  if ((_prefs.manual_add_contacts & 1) == 0) {
    return true;
  }

  uint8_t type_bit = 0;
  switch (contact_type) {
    case ADV_TYPE_CHAT:
      type_bit = AUTO_ADD_CHAT;
      break;
    case ADV_TYPE_REPEATER:
      type_bit = AUTO_ADD_REPEATER;
      break;
    case ADV_TYPE_ROOM:
      type_bit = AUTO_ADD_ROOM_SERVER;
      break;
    case ADV_TYPE_SENSOR:
      type_bit = AUTO_ADD_SENSOR;
      break;
    default:
      return false;  // Unknown type, don't auto-add
  }

  return (_prefs.autoadd_config & type_bit) != 0;
}

bool MyMesh::shouldOverwriteWhenFull() const {
  return (_prefs.autoadd_config & AUTO_ADD_OVERWRITE_OLDEST) != 0;
}

uint8_t MyMesh::getAutoAddMaxHops() const {
  return _prefs.autoadd_max_hops;
}

void MyMesh::onContactOverwrite(const uint8_t* pub_key) {
    _store->deleteBlobByKey(pub_key, PUB_KEY_SIZE); // delete from storage
  if (_serial->isConnected()) {
    out_frame[0] = PUSH_CODE_CONTACT_DELETED;
    memcpy(&out_frame[1], pub_key, PUB_KEY_SIZE);
    _serial->writeFrame(out_frame, 1 + PUB_KEY_SIZE);
  }
}

void MyMesh::onContactsFull() {
  if (_serial->isConnected()) {
    out_frame[0] = PUSH_CODE_CONTACTS_FULL;
    _serial->writeFrame(out_frame, 1);
  }
}

void MyMesh::onDiscoveredContact(ContactInfo &contact, bool is_new, uint8_t path_len, const uint8_t* path) {
  if (_serial->isConnected()) {
    if (is_new) {
      writeContactRespFrame(PUSH_CODE_NEW_ADVERT, contact);
    } else {
      out_frame[0] = PUSH_CODE_ADVERT;
      memcpy(&out_frame[1], contact.id.pub_key, PUB_KEY_SIZE);
      _serial->writeFrame(out_frame, 1 + PUB_KEY_SIZE);
    }
  }

  // add inbound-path to mem cache
  if (path && mesh::Packet::isValidPathLen(path_len)) {  // check path is valid
    AdvertPath* p = advert_paths;
    uint32_t oldest = 0xFFFFFFFF;
    for (int i = 0; i < ADVERT_PATH_TABLE_SIZE; i++) {   // check if already in table, otherwise evict oldest
      if (memcmp(advert_paths[i].pubkey_prefix, contact.id.pub_key, sizeof(AdvertPath::pubkey_prefix)) == 0) {
        p = &advert_paths[i];   // found
        break;
      }
      if (advert_paths[i].recv_timestamp < oldest) {
        oldest = advert_paths[i].recv_timestamp;
        p = &advert_paths[i];
      }
    }

    memcpy(p->pubkey_prefix, contact.id.pub_key, sizeof(p->pubkey_prefix));
    strcpy(p->name, contact.name);
    p->recv_timestamp = getRTCClock()->getCurrentTime();
    p->path_len = mesh::Packet::copyPath(p->path, path, path_len);
  }

  if (_listener) _listener->onDiscoveredContact(contact, is_new, path_len, path);

  if (!is_new) dirty_contacts_expiry = futureMillis(LAZY_CONTACTS_WRITE_DELAY); // only schedule lazy write for contacts that are in contacts[]
}

static int sort_by_recent(const void *a, const void *b) {
  return ((AdvertPath *) b)->recv_timestamp - ((AdvertPath *) a)->recv_timestamp;
}

int MyMesh::getRecentlyHeard(AdvertPath dest[], int max_num) {
  if (max_num > ADVERT_PATH_TABLE_SIZE) max_num = ADVERT_PATH_TABLE_SIZE;
  qsort(advert_paths, ADVERT_PATH_TABLE_SIZE, sizeof(advert_paths[0]), sort_by_recent);

  for (int i = 0; i < max_num; i++) {
    dest[i] = advert_paths[i];
  }
  return max_num;
}


void MyMesh::onContactPathUpdated(const ContactInfo &contact) {
  out_frame[0] = PUSH_CODE_PATH_UPDATED;
  memcpy(&out_frame[1], contact.id.pub_key, PUB_KEY_SIZE);
  _serial->writeFrame(out_frame, 1 + PUB_KEY_SIZE); // NOTE: app may not be connected

  dirty_contacts_expiry = futureMillis(LAZY_CONTACTS_WRITE_DELAY);
}

ContactInfo*  MyMesh::processAck(const uint8_t *data) {
  if (_listener) {
    uint32_t ack_crc;
    memcpy(&ack_crc, data, 4);
    _listener->onACKRecv(ack_crc);
  }
  // see if matches any in a table
  for (int i = 0; i < EXPECTED_ACK_TABLE_SIZE; i++) {
    if (memcmp(data, &expected_ack_table[i].ack, 4) == 0) { // got an ACK from recipient
      out_frame[0] = PUSH_CODE_SEND_CONFIRMED;
      // The app knows the acknowledgement by the hash RESP_CODE_SENT gave it:
      // its own text's, when the text went out transformed.
      if (expected_ack_table[i].app_ack) {
        memcpy(&out_frame[1], &expected_ack_table[i].app_ack, 4);
      } else {
        memcpy(&out_frame[1], data, 4);
      }
      uint32_t trip_time = _ms->getMillis() - expected_ack_table[i].msg_sent;
      memcpy(&out_frame[5], &trip_time, 4);
      _serial->writeFrame(out_frame, 9);

      // NOTE: the same ACK can be received multiple times!
      expected_ack_table[i].ack = 0; // clear expected hash, now that we have received ACK
      expected_ack_table[i].app_ack = 0;
      return expected_ack_table[i].contact;
    }
  }
  return checkConnectionsAck(data);
}

void MyMesh::queueMessage(const ContactInfo &from, uint8_t txt_type, mesh::Packet *pkt,
                          uint32_t sender_timestamp, const uint8_t *extra, int extra_len, const char *text) {
#ifdef WITH_COMPANION_FRAME_FRAGMENTS
  uint8_t frame[companion_frames::MAX_ORIGINAL_LEN];
  int frame_cap = _prefs.framed_packets_enabled ? companion_frames::MAX_ORIGINAL_LEN : MAX_FRAME_SIZE;
#else
  uint8_t* frame = out_frame;
  const int frame_cap = MAX_FRAME_SIZE;
#endif
  int i = 0;
  // Queue in the canonical v3 layout regardless of the connected app; a legacy
  // app (< v3) gets the v3 -> legacy collapse at delivery (CMD_SYNC_NEXT_MESSAGE).
  frame[i++] = RESP_CODE_CONTACT_MSG_RECV_V3;
  frame[i++] = (int8_t)(pkt->getSNR() * 4);
  frame[i++] = 0; // reserved1
  frame[i++] = 0; // reserved2
  memcpy(&frame[i], from.id.pub_key, 6);
  i += 6; // just 6-byte prefix
  // The app gets the packed byte (low 6 bits = hop count, top 2 = hash size - 1),
  // same as upstream: it decodes both fields out of it. Our own display wants
  // just the hop count, or it renders "[64]" for a 2-byte-hash sender.
  frame[i++] = pkt->isRouteFlood() ? pkt->path_len : 0xFF;
  uint8_t path_len = pkt->isRouteFlood() ? pkt->getPathHashCount() : 0xFF;
  frame[i++] = txt_type;
  memcpy(&frame[i], &sender_timestamp, 4);
  i += 4;
  if (extra_len > 0) {
    memcpy(&frame[i], extra, extra_len);
    i += extra_len;
  }
  int tlen = strlen(text); // TODO: UTF-8 ??
  if (i + tlen > frame_cap) tlen = frame_cap - i;
  memcpy(&frame[i], text, tlen);
  i += tlen;
  addToOfflineQueue(frame, i);

  if (_serial->isConnected()) {
    uint8_t notification_frame[1];
    notification_frame[0] = PUSH_CODE_MSG_WAITING; // send push 'tickle'
    _serial->writeFrame(notification_frame, 1);
  }

#ifdef DISPLAY_CLASS
  // we only want to show text messages on display, not cli data
  if (_listener && (txt_type == TXT_TYPE_PLAIN || txt_type == TXT_TYPE_SIGNED_PLAIN)) {
    char display_text[MAX_TEXT_LEN];
    const char* shown = transformCompatText(text, false, display_text, sizeof(display_text))
                            ? display_text : text;
    _listener->onShowMessage(path_len, from.name, shown, offline_queue_len, true);
  }
#endif
  if (_listener) {
    _listener->onMessageRecv(pkt, from, txt_type, sender_timestamp, text);
    _listener->onQueueSizeChanged(offline_queue_len);
  }
}

void MyMesh::onAdvertRecv(mesh::Packet* packet, const mesh::Identity& id, uint32_t timestamp,
                          const uint8_t* app_data, size_t app_data_len) {
  BaseChatMesh::onAdvertRecv(packet, id, timestamp, app_data, app_data_len);

  uint32_t pub_hash;
  memcpy(&pub_hash, id.pub_key, 4);
#ifdef WITH_COMPANION_CLI
  if (_ts_from_adverts)
#endif
  if (_ts.feedAdvert(timestamp, pub_hash)) {
    auto r = _ts.trySync(getRTCClock(), &sensors, hasRecentAppTimeSet());
    if (r == TsSyncResult::GPS) noteTimeSource(TIME_SOURCE_GPS);
    else if (r == TsSyncResult::CLOCK) noteTimeSource(TIME_SOURCE_ADVERT);
  }
}

bool MyMesh::filterRecvFloodPacket(mesh::Packet* packet) {
  // REVISIT: try to determine which Region (from transport_codes[1]) that Sender is indicating for replies/responses
  //    if unknown, fallback to finding Region from transport_codes[0], the 'scope' used by Sender
  return false;
}

bool MyMesh::allowPacketForward(const mesh::Packet* packet) {
  return _prefs.isRepeatEn();
}

void MyMesh::sendFloodScoped(const TransportKey& scope, mesh::Packet* pkt, uint32_t delay_millis) {
  if (scope.isNull()) {
    sendFlood(pkt, delay_millis, _prefs.path_hash_mode + 1);
  } else {
    uint16_t codes[2];
    codes[0] = scope.calcTransportCode(pkt);
    codes[1] = 0;  // REVISIT: set to 'home' Region, for sender/return region?
    sendFlood(pkt, codes, delay_millis, _prefs.path_hash_mode + 1);
  }
}

void MyMesh::sendFloodScoped(const ContactInfo& recipient, mesh::Packet* pkt, uint32_t delay_millis) {
  // TODO: dynamic send_scope, depending on recipient and current 'home' Region
  if (send_unscoped) {
    sendFlood(pkt, delay_millis, _prefs.path_hash_mode + 1);  // app has explicitly requested un-scoped
  } else {
    TransportKey default_scope;
    memcpy(&default_scope.key, _prefs.default_scope_key, sizeof(default_scope.key));

    auto scope = send_scope.isNull() ? &default_scope : &send_scope;
    sendFloodScoped(*scope, pkt, delay_millis);
  }
}
void MyMesh::sendFloodScoped(const mesh::GroupChannel& channel, mesh::Packet* pkt, uint32_t delay_millis) {
  // TODO: have per-channel send_scope
  if (send_unscoped) {
    sendFlood(pkt, delay_millis, _prefs.path_hash_mode + 1);  // app has explicitly requested un-scoped
  } else {
    TransportKey default_scope;
    memcpy(&default_scope.key, _prefs.default_scope_key, sizeof(default_scope.key));

    auto scope = send_scope.isNull() ? &default_scope : &send_scope;
    sendFloodScoped(*scope, pkt, delay_millis);
  }
}

void MyMesh::onMessageRecv(const ContactInfo &from, mesh::Packet *pkt, uint32_t sender_timestamp,
                           const char *text) {
#ifdef WITH_COMPANION_CLI
  if (_ts_from_messages)
#endif
  if (_ts.feedMessage(sender_timestamp, getRTCClock()->getCurrentTime(),
                      TimeSyncHelper::pubHash(from.id.pub_key)) && !hasRecentAppTimeSet()) {
    auto r = _ts.trySync(getRTCClock(), &sensors, false);
    if (r == TsSyncResult::GPS) noteTimeSource(TIME_SOURCE_GPS);
    else if (r == TsSyncResult::CLOCK) noteTimeSource(TIME_SOURCE_ADVERT);
  }
#ifdef WITH_COMPANION_CLI
  if (_prefs.rcli_mode == REMOTE_CLI_PIN || _prefs.rcli_mode == REMOTE_CLI_BOTH) {
    const char* cmd;
    int r = rcliCheckPin(from, text, sender_timestamp, &cmd);
    if (r > 0) {
      rcliAccepted(from);
      handleRemoteCLI(from, sender_timestamp, cmd);
      return;
    }
    if (r < 0) return;   // replayed, stale or locked out: not shown either
  }
#endif
  markConnectionActive(from); // in case this is from a server, and we have a connection
  queueMessage(from, TXT_TYPE_PLAIN, pkt, sender_timestamp, NULL, 0, text);
}

void MyMesh::onCommandDataRecv(const ContactInfo &from, mesh::Packet *pkt, uint32_t sender_timestamp, const char *text) {
  markConnectionActive(from); // in case this is from a server, and we have a connection
  queueMessage(from, TXT_TYPE_CLI_DATA, pkt, sender_timestamp, NULL, 0, text);
}

void MyMesh::onCLICommandRecv(const ContactInfo &from, mesh::Packet *pkt, uint32_t sender_timestamp,
                              const char *text, char* reply) {
  markConnectionActive(from); // in case this is from a server, and we have a connection
#ifdef WITH_COMPANION_CLI
  // The app's CLI screen numbers its commands, "01|get radio", and matches the
  // reply by the same prefix.
  char prefix[4] = {0};
  const char* body = text;
  if (strlen(text) > 4 && text[2] == '|') {
    memcpy(prefix, text, 3);
    body = text + 3;
  }
  const char* cmd = body;
  bool allowed = false;
  const uint8_t mode = _prefs.rcli_mode;
  if (mode == REMOTE_CLI_PIN || mode == REMOTE_CLI_BOTH) {
    int r = rcliCheckPin(from, body, sender_timestamp, &cmd);
    if (r < 0) return;   // no reply: replayed, stale or locked out
    RemoteCliPeer* p = rcliPeer(from);
    if (r > 0) {
      allowed = true;
      rcliAccepted(from);
    } else if (p->session_until && (int32_t)(millis() - p->session_until) < 0) {
      // A command with the PIN opened a session: the next ones may omit it.
      // Only here - in a chat every message would turn into a command.
      if (!rcliFresh(p, sender_timestamp)) return;
      allowed = true;
    }
    if (allowed) {
      p->session_until = millis() + RCLI_SESSION_MS;
      if (!p->session_until) p->session_until = 1;
    }
  }
  if (!allowed && (mode == REMOTE_CLI_FLAG || mode == REMOTE_CLI_BOTH) && from.isRemoteCLIAllowed()) {
    if (!rcliFresh(rcliPeer(from), sender_timestamp)) return;
    cmd = body;
    allowed = true;
  }
  if (allowed) {
    static char cmdBuf[256];
    strncpy(cmdBuf, cmd, sizeof(cmdBuf) - 1);
    cmdBuf[sizeof(cmdBuf) - 1] = '\0';
    char* e = cmdBuf + strlen(cmdBuf);
    while (e > cmdBuf && (e[-1] == ' ' || e[-1] == '\r' || e[-1] == '\n')) *--e = '\0';
    normalizeCliCommand(cmdBuf);

    if (strcmp(cmdBuf, "reboot") == 0) {
      sprintf(reply, "%srebooting in 1s...", prefix);
      _pending_reboot_at = futureMillis(1000);
      _pending_reboot_deadline = futureMillis(30000);
      _app_drain_until = 0;
    } else if (strcmp(cmdBuf, "poweroff") == 0 || strcmp(cmdBuf, "shutdown") == 0) {
      sprintf(reply, "%spowering off...", prefix);
      _pending_poweroff_at = futureMillis(500);
      _pending_poweroff_deadline = futureMillis(30000);
      _app_drain_until = 0;
    } else {
      // The reply goes back in one CLI_DATA packet (BaseChatMesh hands us
      // 161 bytes), but our handlers write up to 512: run them in a buffer of
      // their own and cut the reply to fit.
      static char out[512];
      out[0] = 0;
      if (strcmp(cmdBuf, "logout") == 0) {
        rcliPeer(from)->session_until = 0;
        strcpy(out, "OK - logged out");
      } else {
        runCliChain(sender_timestamp, cmdBuf, out, true);
      }
      strcpy(reply, prefix);
      char* r = reply + strlen(prefix);
      const size_t cap = 160 - strlen(prefix);
      if (strlen(out) > cap) { memcpy(r, out, cap - 3); strcpy(r + cap - 3, "..."); }
      else strcpy(r, out);
    }
    return;
  }
#else
  if (from.isRemoteCLIAllowed()) {
    if (!handleCommand(text, sender_timestamp, reply)) {
      strcat(reply, "Unknown command");   // reply may have cmd prefix from 'text'
    }
    return;
  }
#endif
  queueMessage(from, TXT_TYPE_CLI_COMMAND, pkt, sender_timestamp, NULL, 0, text);
}

void MyMesh::onSignedMessageRecv(const ContactInfo &from, mesh::Packet *pkt, uint32_t sender_timestamp,
                                 const uint8_t *sender_prefix, const char *text) {
  if (_ts.feedMessage(sender_timestamp, getRTCClock()->getCurrentTime(),
                      TimeSyncHelper::pubHash(from.id.pub_key)) && !hasRecentAppTimeSet()) {
    auto r = _ts.trySync(getRTCClock(), &sensors, false);
    if (r == TsSyncResult::GPS) noteTimeSource(TIME_SOURCE_GPS);
    else if (r == TsSyncResult::CLOCK) noteTimeSource(TIME_SOURCE_ADVERT);
  }
  markConnectionActive(from);
  // from.sync_since change needs to be persisted
  dirty_contacts_expiry = futureMillis(LAZY_CONTACTS_WRITE_DELAY);
  queueMessage(from, TXT_TYPE_SIGNED_PLAIN, pkt, sender_timestamp, sender_prefix, 4, text);
}

void MyMesh::onChannelMessageRecv(const mesh::GroupChannel &channel, mesh::Packet *pkt, uint32_t timestamp,
                                  const char *text) {
  if (_ts.feedMessage(timestamp, getRTCClock()->getCurrentTime(),
                      timesync::senderFromChannelText(text)) && !hasRecentAppTimeSet()) {
    auto r = _ts.trySync(getRTCClock(), &sensors, false);
    if (r == TsSyncResult::GPS) noteTimeSource(TIME_SOURCE_GPS);
    else if (r == TsSyncResult::CLOCK) noteTimeSource(TIME_SOURCE_ADVERT);
  }
#ifdef WITH_COMPANION_FRAME_FRAGMENTS
  uint8_t frame[companion_frames::MAX_ORIGINAL_LEN];
  int frame_cap = _prefs.framed_packets_enabled ? companion_frames::MAX_ORIGINAL_LEN : MAX_FRAME_SIZE;
#else
  uint8_t* frame = out_frame;
  const int frame_cap = MAX_FRAME_SIZE;
#endif
  int i = 0;
  // Queue in the canonical v3 layout (see queueMessage).
  frame[i++] = RESP_CODE_CHANNEL_MSG_RECV_V3;
  frame[i++] = (int8_t)(pkt->getSNR() * 4);
  frame[i++] = 0; // reserved1
  frame[i++] = 0; // reserved2

  uint8_t channel_idx = findChannelIdx(channel);
  {
    ChannelDetails ch;
    if (getChannel(channel_idx, ch) && strcmp(ch.name, "TerminalCLI") == 0) return;
  }
  frame[i++] = channel_idx;
  // The app gets the packed byte (low 6 bits = hop count, top 2 = hash size - 1),
  // same as upstream: it decodes both fields out of it. Our own display wants
  // just the hop count, or it renders "[64]" for a 2-byte-hash sender.
  frame[i++] = pkt->isRouteFlood() ? pkt->path_len : 0xFF;
  uint8_t path_len = pkt->isRouteFlood() ? pkt->getPathHashCount() : 0xFF;

  frame[i++] = TXT_TYPE_PLAIN;
  memcpy(&frame[i], &timestamp, 4);
  i += 4;
  int tlen = strlen(text); // TODO: UTF-8 ??
  if (i + tlen > frame_cap) tlen = frame_cap - i;
  memcpy(&frame[i], text, tlen);
  i += tlen;
  const int queued_at = addToOfflineQueue(frame, i);
#if WITH_OFFLINE_ROUTES
  keepRoute(queued_at, pkt);
#else
  (void)queued_at;
#endif

  if (_serial->isConnected()) {
    uint8_t notification_frame[1];
    notification_frame[0] = PUSH_CODE_MSG_WAITING; // send push 'tickle'
    _serial->writeFrame(notification_frame, 1);
  }
  if (_listener) {
    // Get the channel name from the channel index
    ChannelDetails channel_details;
    if (!getChannel(channel_idx, channel_details)) {
      strcpy(channel_details.name, "Unknown");
    }
#ifdef DISPLAY_CLASS
    char display_text[MAX_TEXT_LEN];
    const char* shown = transformCompatText(text, true, display_text, sizeof(display_text))
                            ? display_text : text;
    _listener->onShowMessage(path_len, channel_details.name, shown, offline_queue_len, false);
#endif
    _listener->onChannelMessageRecv(pkt, channel_details, text);
    _listener->onQueueSizeChanged(offline_queue_len);
  }
}

void MyMesh::onChannelDataRecv(const mesh::GroupChannel &channel, mesh::Packet *pkt, uint16_t data_type,
                               const uint8_t *data, size_t data_len) {
  if (data_len > MAX_CHANNEL_DATA_LENGTH) {
    MESH_DEBUG_PRINTLN("onChannelDataRecv: dropping payload_len=%d exceeds frame limit=%d",
                       (uint32_t)data_len, (uint32_t)MAX_CHANNEL_DATA_LENGTH);
    return;
  }

  int i = 0;
  out_frame[i++] = RESP_CODE_CHANNEL_DATA_RECV;
  out_frame[i++] = (int8_t)(pkt->getSNR() * 4);
  out_frame[i++] = 0; // reserved1
  out_frame[i++] = 0; // reserved2

  uint8_t channel_idx = findChannelIdx(channel);
  out_frame[i++] = channel_idx;
  out_frame[i++] = pkt->isRouteFlood() ? pkt->path_len : 0xFF;  // packed byte, as above
  out_frame[i++] = (uint8_t)(data_type & 0xFF);
  out_frame[i++] = (uint8_t)(data_type >> 8);
  out_frame[i++] = (uint8_t)data_len;

  int copy_len = (int)data_len;
  if (copy_len > 0) {
    memcpy(&out_frame[i], data, copy_len);
    i += copy_len;
  }

  uint8_t compat_flags = 0;
#ifdef WITH_AEIC_DETECT
  const aeic::ChunkInfo aeic_info = aeic::parseChunk(data_type, data, data_len);
  const bool first_aeic_chunk = (aeic_info.isData() || aeic_info.isParity()) &&
      markAEICNotice(channel_idx, aeic_info.sender_prefix, aeic_info.image_id,
                     aeic_info.total, (uint32_t)millis());
  if (first_aeic_chunk) compat_flags |= FRAME_COMPAT_AEIC_NOTICE;
  const bool aeic_notice = isAEICDetectEnabled() && first_aeic_chunk;
#endif

  // Store only the original binary frame. Any compatibility text is derived
  // later, when the connected app requests this frame and its capabilities
  // are known.
  const int queued_at = addToOfflineQueue(out_frame, i, compat_flags);
#if WITH_OFFLINE_ROUTES
  keepRoute(queued_at, pkt);
#else
  (void)queued_at;
#endif

#ifdef DISPLAY_CLASS
  const uint8_t hop_count = pkt->isRouteFlood() ? pkt->getPathHashCount() : 0xFF;
  const char* channel_name = "Unknown";
  ChannelDetails channel_details;
  if (getChannel(channel_idx, channel_details)) channel_name = channel_details.name;
#endif

#if defined(WITH_MCMP_DETECT) && defined(DISPLAY_CLASS)
  mcmp::Meta mcmp_meta;
  if (isMCMPDetectEnabled() &&
      mcmp::parseBinaryEnvelope(data_type, data, data_len, mcmp_meta)) {
    char placeholder[96];
    const int placeholder_length = mcmp::formatPlaceholder(
        mcmp_meta, placeholder, sizeof(placeholder));
    const char* sender = mcmp_meta.has_sender ? mcmp_meta.sender : "MCMP";
    if (_listener && placeholder_length > 0) {
      char display_text[144];
      snprintf(display_text, sizeof(display_text), "%s: %s", sender, placeholder);
      _listener->onShowMessage(hop_count, channel_name, display_text, offline_queue_len, false);
    }
  }
#endif

#ifdef WITH_AEIC_DETECT
  if (aeic_notice) {
    char sender[32];
    aeicSenderName(aeic_info.sender_prefix, sender, sizeof(sender));
    char notice[64];
    snprintf(notice, sizeof(notice), "%s: <AEIC image>", sender);
#ifdef DISPLAY_CLASS
    if (_listener) _listener->onShowMessage(hop_count, channel_name, notice, offline_queue_len, false);
#endif
  }
#endif

#if defined(WITH_MCOIMG_DETECT) && defined(DISPLAY_CLASS)
  mcoimg_detect::Meta image_meta;
  if (isMCOimgDetectEnabled() &&
      mcoimg_detect::parseBinaryEnvelope(data_type, data, data_len, image_meta)) {
    char placeholder[96];
    snprintf(placeholder, sizeof(placeholder), "%s: <MCOimg v%u image>",
             image_meta.sender[0] ? image_meta.sender : "MCOimg",
             (unsigned)image_meta.version);
    if (_listener) _listener->onShowMessage(hop_count, channel_name, placeholder, offline_queue_len, false);
  }
#endif

#ifdef WITH_MCOTXT
#ifdef DISPLAY_CLASS
  if (_listener && isMCOtxtEnabled() && mcotxt::isBinaryEnvelope(data_type, data, data_len)) {
    mcotxt::DecodedMessage message;
    char* decoded = mcotxt::scratch();
    const mcotxt::MessageStatus status = mcotxt::decodeBinaryEnvelope(
        data_type, data, data_len, decoded, mcotxt::kScratchBytes, message);
    if (status == mcotxt::MessageStatus::Ok || status == mcotxt::MessageStatus::TooLong) {
      bool mention_truncated = false;
      mcotxt::ensureReplyMentionPrefix(message, decoded, mcotxt::kScratchBytes,
                                       mention_truncated);
      char display_text[MAX_TEXT_LEN];
      snprintf(display_text, sizeof(display_text), "%s: %s",
               message.has_sender ? message.sender : "?", decoded);
      _listener->onShowMessage(hop_count, channel_name, display_text, offline_queue_len, false);
    }
  }
#endif
#endif

  if (_serial->isConnected()) {
    uint8_t frame[1];
    frame[0] = PUSH_CODE_MSG_WAITING; // send push 'tickle'
    _serial->writeFrame(frame, 1);
  }
  if (_listener) {
    _listener->onChannelDataRecv(pkt, channel, data_type, data, data_len);
  }
}

uint8_t MyMesh::onContactRequest(const ContactInfo &contact, uint32_t sender_timestamp, const uint8_t *data,
                                 uint8_t len, uint8_t *reply) {
  if (data[0] == REQ_TYPE_GET_TELEMETRY_DATA) {
    uint8_t permissions = 0;
    uint8_t cp = contact.flags >> 1; // LSB used as 'favourite' bit (so only use upper bits)

    if (_prefs.telemetry_mode_base == TELEM_MODE_ALLOW_ALL) {
      permissions = TELEM_PERM_BASE;
    } else if (_prefs.telemetry_mode_base == TELEM_MODE_ALLOW_FLAGS) {
      permissions = cp & TELEM_PERM_BASE;
    }

    if (_prefs.telemetry_mode_loc == TELEM_MODE_ALLOW_ALL) {
      permissions |= TELEM_PERM_LOCATION;
    } else if (_prefs.telemetry_mode_loc == TELEM_MODE_ALLOW_FLAGS) {
      permissions |= cp & TELEM_PERM_LOCATION;
    }

    if (_prefs.telemetry_mode_env == TELEM_MODE_ALLOW_ALL) {
      permissions |= TELEM_PERM_ENVIRONMENT;
    } else if (_prefs.telemetry_mode_env == TELEM_MODE_ALLOW_FLAGS) {
      permissions |= cp & TELEM_PERM_ENVIRONMENT;
    }

    uint8_t perm_mask = ~(data[1]);    // NEW: first reserved byte (of 4), is now inverse mask to apply to permissions
    permissions &= perm_mask;

    if (permissions & TELEM_PERM_BASE) { // only respond if base permission bit is set
      telemetry.reset();
      telemetry.addVoltage(TELEM_CHANNEL_SELF, (float)board.getBattMilliVolts() / 1000.0f);
      // query other sensors -- target specific
      sensors.querySensors(permissions, telemetry);

      float temperature = board.getMCUTemperature();
      if(!isnan(temperature)) { // Supported boards with built-in temperature sensor. ESP32-C3 may return NAN
        telemetry.addTemperature(TELEM_CHANNEL_SELF, temperature); // Built-in MCU Temperature
      }

      memcpy(reply, &sender_timestamp,
             4); // reflect sender_timestamp back in response packet (kind of like a 'tag')

      uint8_t tlen = telemetry.getSize();
      memcpy(&reply[4], telemetry.getBuffer(), tlen);
      return 4 + tlen;
    }
  } else if (_listener) {
    return _listener->onUnhandledRequest(contact, sender_timestamp, data, len, reply);
  }
  return 0; // unknown
}

void MyMesh::onContactResponse(const ContactInfo &contact, const uint8_t *data, uint8_t len) {
  uint32_t tag;
  memcpy(&tag, data, 4);

  if (pending_login && memcmp(&pending_login, contact.id.pub_key, 4) == 0) { // check for login response
    // yes, is response to pending sendLogin()
    pending_login = 0;

    int i = 0;
    if (memcmp(&data[4], "OK", 2) == 0) { // legacy Repeater login OK response
      out_frame[i++] = PUSH_CODE_LOGIN_SUCCESS;
      out_frame[i++] = 0; // legacy: is_admin = false
      memcpy(&out_frame[i], contact.id.pub_key, 6);
      i += 6;                                     // pub_key_prefix
    } else if (data[4] == RESP_SERVER_LOGIN_OK) { // new login response
      uint16_t keep_alive_secs = ((uint16_t)data[5]) * 16;
      if (keep_alive_secs > 0) {
        startConnection(contact, keep_alive_secs);
      }
      out_frame[i++] = PUSH_CODE_LOGIN_SUCCESS;
      out_frame[i++] = data[6]; // permissions (eg. is_admin)
      memcpy(&out_frame[i], contact.id.pub_key, 6);
      i += 6; // pub_key_prefix
      memcpy(&out_frame[i], &tag, 4);
      i += 4; // NEW: include server timestamp
      out_frame[i++] = data[7]; // NEW (v7): ACL permissions
      out_frame[i++] = data[12]; // FIRMWARE_VER_LEVEL
    } else {
      out_frame[i++] = PUSH_CODE_LOGIN_FAIL;
      out_frame[i++] = 0; // reserved
      memcpy(&out_frame[i], contact.id.pub_key, 6);
      i += 6; // pub_key_prefix
    }
    _serial->writeFrame(out_frame, i);
  } else if (len > 4 && // check for status response
             pending_status &&
             memcmp(&pending_status, contact.id.pub_key, 4) == 0 // legacy matching scheme
                                                                 // FUTURE: tag == pending_status
  ) {
    pending_status = 0;

    int i = 0;
    out_frame[i++] = PUSH_CODE_STATUS_RESPONSE;
    out_frame[i++] = 0; // reserved
    memcpy(&out_frame[i], contact.id.pub_key, 6);
    i += 6; // pub_key_prefix
    memcpy(&out_frame[i], &data[4], len - 4);
    i += (len - 4);
    _serial->writeFrame(out_frame, i);
  } else if (len > 4 && tag == pending_telemetry) {  // check for matching response tag
    pending_telemetry = 0;

    int i = 0;
    out_frame[i++] = PUSH_CODE_TELEMETRY_RESPONSE;
    out_frame[i++] = 0; // reserved
    memcpy(&out_frame[i], contact.id.pub_key, 6);
    i += 6; // pub_key_prefix
    memcpy(&out_frame[i], &data[4], len - 4);
    i += (len - 4);
    _serial->writeFrame(out_frame, i);
  } else if (len > 4 && tag == pending_req) {  // check for matching response tag
    pending_req = 0;

    int i = 0;
    out_frame[i++] = PUSH_CODE_BINARY_RESPONSE;
    out_frame[i++] = 0; // reserved
    memcpy(&out_frame[i], &tag, 4);   // app needs to match this to RESP_CODE_SENT.tag
    i += 4;
    memcpy(&out_frame[i], &data[4], len - 4);
    i += (len - 4);
    _serial->writeFrame(out_frame, i);
  } else {
    bool handled;
    if (_listener && len > 4) {
      handled = _listener->onUnhandledResponse(contact, tag, &data[4], len - 4);
    } else {
      handled = false;
    }
    if (!handled) {   // let the app know about unknown/unhandled responses (eg. push telemetry packets)
      int i = 0;
      out_frame[i++] = PUSH_CODE_BINARY_RESPONSE;
      out_frame[i++] = 0; // reserved
      memcpy(&out_frame[i], data, len);
      i += len;
      _serial->writeFrame(out_frame, i);
    }
  }
}

bool MyMesh::onContactPathRecv(ContactInfo& contact, uint8_t* in_path, uint8_t in_path_len, uint8_t* out_path, uint8_t out_path_len, uint8_t extra_type, uint8_t* extra, uint8_t extra_len) {
  if (extra_type == PAYLOAD_TYPE_RESPONSE && extra_len > 4) {
    uint32_t tag;
    memcpy(&tag, extra, 4);

    if (tag == pending_discovery) {  // check for matching response tag)
      pending_discovery = 0;

      if (!mesh::Packet::isValidPathLen(in_path_len) || !mesh::Packet::isValidPathLen(out_path_len)) {
        MESH_DEBUG_PRINTLN("onContactPathRecv, invalid path sizes: %d, %d", in_path_len, out_path_len);
      } else {
        int i = 0;
        out_frame[i++] = PUSH_CODE_PATH_DISCOVERY_RESPONSE;
        out_frame[i++] = 0; // reserved
        memcpy(&out_frame[i], contact.id.pub_key, 6);
        i += 6; // pub_key_prefix
        out_frame[i++] = out_path_len;
        i += mesh::Packet::writePath(&out_frame[i], out_path, out_path_len);
        out_frame[i++] = in_path_len;
        i += mesh::Packet::writePath(&out_frame[i], in_path, in_path_len);
        // NOTE: telemetry data in 'extra' is discarded at present

        _serial->writeFrame(out_frame, i);
      }
      return false;  // DON'T send reciprocal path!
    }
  }
  // let base class handle received path and data
  return BaseChatMesh::onContactPathRecv(contact, in_path, in_path_len, out_path, out_path_len, extra_type, extra, extra_len);
}

void MyMesh::onControlDataRecv(mesh::Packet *packet) {
#ifdef WITH_COMPANION_FRAME_FRAGMENTS
  writePacketPayloadFrameMaybeFragmented(packet, PUSH_CODE_CONTROL_DATA, packet->path_len, "onControlDataRecv");
#else
  if (packet->payload_len + 4 > sizeof(out_frame)) {
    MESH_DEBUG_PRINTLN("onControlDataRecv(), payload_len too long: %d", packet->payload_len);
    return;
  }
  int i = 0;
  out_frame[i++] = PUSH_CODE_CONTROL_DATA;
  out_frame[i++] = (int8_t)(_radio->getLastSNR() * 4);
  out_frame[i++] = (int8_t)(_radio->getLastRSSI());
  out_frame[i++] = packet->path_len;
  memcpy(&out_frame[i], packet->payload, packet->payload_len);
  i += packet->payload_len;

  if (_serial->isConnected()) {
    _serial->writeFrame(out_frame, i);
  } else {
    MESH_DEBUG_PRINTLN("onControlDataRecv(), data received while app offline");
  }
#endif

  if (_listener) _listener->onControlDataRecv(packet);
}

void MyMesh::onRawDataRecv(mesh::Packet *packet) {
#ifdef WITH_COMPANION_FRAME_FRAGMENTS
  writePacketPayloadFrameMaybeFragmented(packet, PUSH_CODE_RAW_DATA, 0xFF, "onRawDataRecv");
#else
  if (packet->payload_len + 4 > sizeof(out_frame)) {
    MESH_DEBUG_PRINTLN("onRawDataRecv(), payload_len too long: %d", packet->payload_len);
    return;
  }
  int i = 0;
  out_frame[i++] = PUSH_CODE_RAW_DATA;
  out_frame[i++] = (int8_t)(_radio->getLastSNR() * 4);
  out_frame[i++] = (int8_t)(_radio->getLastRSSI());
  out_frame[i++] = 0xFF; // reserved (possibly path_len in future)
  memcpy(&out_frame[i], packet->payload, packet->payload_len);
  i += packet->payload_len;

  if (_serial->isConnected()) {
    _serial->writeFrame(out_frame, i);
  } else {
    MESH_DEBUG_PRINTLN("onRawDataRecv(), data received while app offline");
  }
#endif
  if (_listener) {
    _listener->onRawDataRecv(packet);
  }
}

void MyMesh::onTraceRecv(mesh::Packet *packet, uint32_t tag, uint32_t auth_code, uint8_t flags,
                         const uint8_t *path_snrs, const uint8_t *path_hashes, uint8_t path_len) {
  uint8_t path_sz = flags & 0x03;  // NEW v1.11+
  if (12 + path_len + (path_len >> path_sz) + 1 > sizeof(out_frame)) {
    MESH_DEBUG_PRINTLN("onTraceRecv(), path_len is too long: %d", (uint32_t)path_len);
    return;
  }
  int i = 0;
  out_frame[i++] = PUSH_CODE_TRACE_DATA;
  out_frame[i++] = 0; // reserved
  out_frame[i++] = path_len;
  out_frame[i++] = flags;
  memcpy(&out_frame[i], &tag, 4);
  i += 4;
  memcpy(&out_frame[i], &auth_code, 4);
  i += 4;
  memcpy(&out_frame[i], path_hashes, path_len);
  i += path_len;

  memcpy(&out_frame[i], path_snrs, path_len >> path_sz);
  i += path_len >> path_sz;
  out_frame[i++] = (int8_t)(packet->getSNR() * 4); // extra/final SNR (to this node)

  if (_serial->isConnected()) {
    _serial->writeFrame(out_frame, i);
  } else {
    MESH_DEBUG_PRINTLN("onTraceRecv(), data received while app offline");
  }
  if (_listener) {
    _listener->onTraceRecv(packet, tag, auth_code, flags, path_snrs, path_hashes, path_len);
  }
}

uint32_t MyMesh::calcFloodTimeoutMillisFor(uint32_t pkt_airtime_millis) const {
  return SEND_TIMEOUT_BASE_MILLIS + (FLOOD_SEND_TIMEOUT_FACTOR * pkt_airtime_millis);
}
uint32_t MyMesh::calcDirectTimeoutMillisFor(uint32_t pkt_airtime_millis, uint8_t path_len) const {
  uint8_t path_hash_count = path_len & 63;
  return SEND_TIMEOUT_BASE_MILLIS +
         ((pkt_airtime_millis * DIRECT_SEND_PERHOP_FACTOR + DIRECT_SEND_PERHOP_EXTRA_MILLIS) *
          (path_hash_count + 1));
}

void MyMesh::onSendTimeout() {}

MyMesh::MyMesh(mesh::Radio &radio, mesh::RNG &rng, mesh::RTCClock &rtc, SimpleMeshTables &tables, DataStore& store)
    : BaseChatMesh(radio, *new ArduinoMillis(), rng, rtc, *new StaticPoolPacketManager(16), tables),
      _serial(NULL), telemetry(MAX_PACKET_PAYLOAD - 4), _store(&store), _listener(NULL), _iter(0),
      offline_store(offline_fs, OFFLINE_STORE_SLOTS, offline_slot_seq, offline_scratch,
                    sizeof(offline_scratch), OFFLINE_STORE_MAX_FRAME, OFFLINE_ROUTE_MAX) {
  _iter_started = false;
  _cli_rescue = false;
  cli_command[0] = 0;
  offline_queue_len = 0;
  offline_fs.store = &store;
  offline_last_enqueue = 0;
  offline_pending_since = 0;
  offline_last_restored = 0;
  offline_last_lost = 0;
  offline_beat = 0;
  for (int i = 0; i < OFFLINE_QUEUE_SIZE; i++) clearQueueAux(i);
#if WITH_OFFLINE_ROUTES
  for (int i = 0; i < OFFLINE_ROUTE_POOL_SLOTS; i++) route_pool[i].seq = offline_ring::NO_SEQ;
  app_session = 0;
#endif
  resetAppSyncState();
#ifdef WITH_COMPANION_FRAME_FRAGMENTS
  _app_supports_frmfrg1 = false;
  next_fragment_id = 1;
  long_frame_slot_mask = 0;
  resetFragmentTransferState();
#endif
  app_target_ver = 0;
  clearPendingReqs();
  next_ack_idx = 0;
  sign_data = NULL;
  dirty_contacts_expiry = 0;
  dirty_prefs_expiry = 0;
  memset(advert_paths, 0, sizeof(advert_paths));
  memset(send_scope.key, 0, sizeof(send_scope.key));
  send_unscoped = false;

  // defaults
  _prefs.airtime_factor = 1.0;
  _prefs.tx_delay_factor = 0.5f;
  _prefs.direct_tx_delay_factor = 0.2f;
  strcpy(_prefs.node_name, "NONAME");
  _prefs.freq = LORA_FREQ;
  _prefs.sf = LORA_SF;
  _prefs.bw = LORA_BW;
  _prefs.cr = LORA_CR;
  _prefs.tx_power_dbm = LORA_TX_POWER;
  _prefs.gps_enabled = 0;       // GPS disabled by default
  _prefs.gps_interval = 0;      // No automatic GPS updates by default
  _prefs.ui_pm_clock_mode = 1;  // PM inline on clock page by default
  _prefs.ui_clock_dim_mode = 0; // normal auto-off by default
  _prefs.adc_multiplier = 0.0f; // 0.0f = use board default
#ifdef EINK_DEFAULT_ROTATION
  _prefs.ui_display_rotation = EINK_DEFAULT_ROTATION;   // GxEPDDisplay takes rotation from prefs (async screen_180)
#else
  _prefs.ui_display_rotation = 3;   // default landscape (matches compile-time DISPLAY_ROTATION=3)
#endif
  _prefs.ui_max_unread_idx = 1;     // 32 unread messages
  _prefs.ui_max_log_idx = 1;        // 32 history messages
  _prefs.ui_charge_uptime_base = 0;
  _prefs.ui_mcotxt_disabled = 0;
  _prefs.ui_mcmp_detect_off = 0;
  _prefs.ui_aeic_detect_off = 0;
  _prefs.ui_mcoimg_detect_off = 0;
  _prefs.radio_fem_rxgain = 1;
#ifdef RADIO_FEM_TXGAIN
  _prefs.radio_fem_txgain = RADIO_FEM_TXGAIN;   // board-specific default for PA
#else
  _prefs.radio_fem_txgain = 0;
#endif
  //_prefs.rx_delay_base = 10.0f;  enable once new algo fixed
  _prefs.setRepeatEn(false);
#if defined(USE_SX1262) || defined(USE_SX1268)
#ifdef SX126X_RX_BOOSTED_GAIN
  _prefs.rx_boosted_gain = SX126X_RX_BOOSTED_GAIN;
#else
  _prefs.rx_boosted_gain = 1; // enabled by default
#endif
#endif
}

void MyMesh::noteTimeSource(TimeSource source) {
  _time_source = source;
  if (_listener) _listener->onStatusChanged();
}

bool MyMesh::hasRecentAppTimeSet() const {
  uint32_t now = getRTCClock()->getCurrentTime();
  return _app_time_lock_until != 0 && now < _app_time_lock_until;
}

const char *MyMesh::getTimeSourceLabel() const {
  switch (_time_source) {
    case TIME_SOURCE_CONTACTS: return "CNT";
    case TIME_SOURCE_ADVERT:   return "ADV";
    case TIME_SOURCE_APP:      return "APP";
    case TIME_SOURCE_RTC:      return "RTC";
    case TIME_SOURCE_GPS:      return "GPS";
    case TIME_SOURCE_UNSET:
    default:                   return "---";
  }
}

void MyMesh::begin() {
  BaseChatMesh::begin();
  noteTimeSource(getRTCClock()->isTimeReliable() ? TIME_SOURCE_RTC : TIME_SOURCE_UNSET);

  if (!_store->loadMainIdentity(self_id)) {
    self_id = radio_new_identity(); // create new random identity
    int count = 0;
    while (count < 10 && (self_id.pub_key[0] == 0x00 || self_id.pub_key[0] == 0xFF)) { // reserved id hashes
      self_id = radio_new_identity();
      count++;
    }
    _store->saveMainIdentity(self_id);
  }

// if name is provided as a build flag, use that as default node name instead
#ifdef ADVERT_NAME
  strcpy(_prefs.node_name, ADVERT_NAME);
#else
  // use hex of first 4 bytes of identity public key as default node name
  char pub_key_hex[10];
  mesh::Utils::toHex(pub_key_hex, self_id.pub_key, 4);
  strcpy(_prefs.node_name, pub_key_hex);
#endif

  // if build provides default-scope, init with that
#ifdef DEFAULT_FLOOD_SCOPE_NAME
  strcpy(_prefs.default_scope_name, DEFAULT_FLOOD_SCOPE_NAME);
  {
    TransportKeyStore temp;
    TransportKey key;
    temp.getAutoKeyFor(0, "#" DEFAULT_FLOOD_SCOPE_NAME, key);
    memcpy(_prefs.default_scope_key, key.key, sizeof(key.key));
  }
#endif

  // load persisted prefs
  _store->loadPrefs(_prefs);
  sensors.node_lat = _prefs.node_lat;
  sensors.node_lon = _prefs.node_lon;

  // sanitise bad pref values
  _prefs.rx_delay_base = constrain(_prefs.rx_delay_base, 0, 20.0f);
  _prefs.tx_delay_factor = constrain(_prefs.tx_delay_factor, 0, 2.0f);
  _prefs.direct_tx_delay_factor = constrain(_prefs.direct_tx_delay_factor, 0, 2.0f);
  _prefs.airtime_factor = constrain(_prefs.airtime_factor, 0, 9.0f);
  _prefs.freq = constrain(_prefs.freq, 150.0f, 2500.0f);
  _prefs.bw = constrain(_prefs.bw, 7.8f, 500.0f);
  _prefs.sf = constrain(_prefs.sf, 5, 12);
  _prefs.cr = constrain(_prefs.cr, 5, 8);
  _prefs.tx_power_dbm = constrain(_prefs.tx_power_dbm, -9, MAX_LORA_TX_POWER);
  _prefs.gps_enabled = constrain(_prefs.gps_enabled, 0, 1);  // Ensure boolean 0 or 1
  _prefs.gps_interval = constrain(_prefs.gps_interval, 0, 86400);  // Max 24 hours
  _prefs.cyr2lat_channels = constrain(_prefs.cyr2lat_channels, 0, 1);
  _prefs.cyr2lat_contacts = constrain(_prefs.cyr2lat_contacts, 0, 1);
  _prefs.ui_pm_clock_mode = constrain(_prefs.ui_pm_clock_mode, 0, 1);
  _prefs.ui_clock_dim_mode = constrain(_prefs.ui_clock_dim_mode, 0, 1);
  _prefs.adc_multiplier = constrain(_prefs.adc_multiplier, 0.0f, 10.0f);
  board.setAdcMultiplier(_prefs.adc_multiplier);
  _prefs.ui_display_rotation = constrain(_prefs.ui_display_rotation, 0, 3);
  _prefs.ui_max_unread_idx = constrain(_prefs.ui_max_unread_idx, 0, 2);
  _prefs.ui_max_log_idx = constrain(_prefs.ui_max_log_idx, 0, 2);
  _prefs.ui_mcotxt_disabled = constrain(_prefs.ui_mcotxt_disabled, 0, 1);
  _prefs.ui_mcmp_detect_off = constrain(_prefs.ui_mcmp_detect_off, 0, 1);
  _prefs.ui_aeic_detect_off = constrain(_prefs.ui_aeic_detect_off, 0, 1);
  _prefs.ui_mcoimg_detect_off = constrain(_prefs.ui_mcoimg_detect_off, 0, 1);
#ifdef WITH_COMPANION_FRAME_FRAGMENTS
  _prefs.framed_packets_enabled = constrain(_prefs.framed_packets_enabled, 0, 1);
#endif
  _cyr2lat_channels_enabled = _prefs.cyr2lat_channels != 0;
  _cyr2lat_contacts_enabled = _prefs.cyr2lat_contacts != 0;

  _active_ble_pin = _prefs.ble_pin;

  resetContacts();
  _store->loadContacts(this);
  // setCurrentTime() marks the clock reliable, so ask before bootstrapping.
  const bool rtc_was_reliable = getRTCClock()->isTimeReliable();
  uint32_t rtc_before_contacts = getRTCClock()->getCurrentTime();
  if (bootstrapRTCfromContacts() > 0) {
    dirty_contacts_expiry = futureMillis(LAZY_CONTACTS_WRITE_DELAY);   // persist repaired lastmod values
  }
  uint32_t rtc_after_contacts = getRTCClock()->getCurrentTime();
  if (!rtc_was_reliable && rtc_after_contacts > rtc_before_contacts &&
      rtc_after_contacts > MIN_VALID_TS && rtc_after_contacts < MAX_VALID_TS) {
    noteTimeSource(TIME_SOURCE_CONTACTS);
    // The newest contact's time is only a starting point, possibly hours old:
    // the first quorum must still be able to step it in either direction with
    // the unset-clock rules, not the seven senders and max_jump of a synced one.
    _ts.clock_restored = true;
  }
  addChannel("Public", PUBLIC_GROUP_PSK); // pre-configure public channel
  _store->loadChannels(this);

  radio_driver.setParams(_prefs.freq, _prefs.bw, _prefs.sf, _prefs.cr);
  radio_driver.setTxPower(_prefs.tx_power_dbm);
  radio_driver.setRxBoostedGainMode(_prefs.rx_boosted_gain);

  board.attachDynamicPrefs(_prefs.getCustom());

  MESH_DEBUG_PRINTLN("RX Boosted Gain Mode: %s",
                     radio_driver.getRxBoostedGainMode() ? "Enabled" : "Disabled");

#ifdef WITH_COMPANION_CLI
  _cli_cb = new CompanionCLICallbacks(*this, *_store);
  _cli    = new CommonCLI(board, *getRTCClock(), sensors, &_prefs, _cli_cb);
  ensureCliPin();
  Serial.printf("CLI PIN: %s\n", _prefs.cli_pin);

  // auto-register TerminalCLI channel if not present
  {
    bool found = false;
    ChannelDetails ch;
    for (int i = 0; i < MAX_GROUP_CHANNELS && !found; i++) {
      if (getChannel(i, ch) && ch.name[0] && strcmp(ch.name, "TerminalCLI") == 0)
        found = true;
    }
    if (found) {
      MESH_DEBUG_PRINTLN("CLI: TerminalCLI channel already present");
    } else if (addChannel("TerminalCLI", TERMINAL_CLI_PSK)) {
      MESH_DEBUG_PRINTLN("CLI: TerminalCLI channel registered");
      saveChannels();
    } else {
      MESH_DEBUG_PRINTLN("CLI: ERROR — addChannel(TerminalCLI) failed (slots full or bad PSK)");
    }
  }
#endif

  beginOfflineStore();
}

const char *MyMesh::getNodeName() {
  return _prefs.node_name;
}
NodePrefs *MyMesh::getNodePrefs() {
  return &_prefs;
}
uint32_t MyMesh::getBLEPin() {
  return _active_ble_pin;
}
void MyMesh::setBLEPin(uint32_t active_pin) {
  _active_ble_pin = active_pin;
}

void MyMesh::deferSavePrefs() {
  dirty_prefs_expiry = futureMillis(LAZY_PREFS_WRITE_DELAY);
}

struct FreqRange {
  uint32_t lower_freq, upper_freq;
};

static FreqRange repeat_freq_ranges[] = {
  #ifdef ALLOWED_REPEAT_FREQ_RANGE
  ALLOWED_REPEAT_FREQ_RANGE
  #else
  { 433000, 433000 },
  { 869495, 869495 },
  { 918000, 918000 }
  #endif
};

bool MyMesh::isValidClientRepeatFreq(uint32_t f) const {
  for (int i = 0; i < sizeof(repeat_freq_ranges)/sizeof(repeat_freq_ranges[0]); i++) {
    auto r = &repeat_freq_ranges[i];
    if (f >= r->lower_freq && f <= r->upper_freq) return true;
  }
  return false;
}

void MyMesh::startInterface(BaseSerialInterface &serial) {
  _app_info_len = 0;
#ifdef WITH_COMPANION_FRAME_FRAGMENTS
  resetFragmentSession();
#else
  resetAppSyncState();
#endif
  _serial = &serial;
  serial.enable();
  _serial_was_connected = serial.isConnected();
}

void MyMesh::handleCmdFrame(size_t len) {
  if (cmd_frame[0] == CMD_DEVICE_QUERY && len >= 2) { // sent when app establishes connection
    app_target_ver = cmd_frame[1];                    // which version of protocol does app understand

    int i = 0;
    out_frame[i++] = RESP_CODE_DEVICE_INFO;
    out_frame[i++] = FIRMWARE_VER_CODE;
    out_frame[i++] = MAX_CONTACTS / 2;   // v3+
    out_frame[i++] = MAX_GROUP_CHANNELS; // v3+
    memcpy(&out_frame[i], &_prefs.ble_pin, 4);
    i += 4;
    memset(&out_frame[i], 0, 12);
    strcpy((char *)&out_frame[i], FIRMWARE_BUILD_DATE);
    i += 12;
    StrHelper::strzcpy((char *)&out_frame[i], board.getManufacturerName(), 40);
    i += 40;
    StrHelper::strzcpy((char *)&out_frame[i], FIRMWARE_VERSION, 20);
    i += 20;
    out_frame[i++] = _prefs.isRepeatEn() ? 1 : 0;   // v9+
    out_frame[i++] = _prefs.path_hash_mode;  // v10+
    _serial->writeFrame(out_frame, i);
  } else if (cmd_frame[0] == CMD_APP_START &&
             len >= 8) { // sent when app establishes connection, respond with node ID
    //  cmd_frame[1..7]  reserved future
    char *app_name = (char *)&cmd_frame[8];
    cmd_frame[len] = 0; // make app_name null terminated
    // Keep the bytes after the 8-byte header (app name and its capability list)
    // for offline_route::deliver(): truncate if longer than the buffer.
    const size_t app_info_len = len - 8;
    _app_info_len = (uint8_t)(app_info_len > sizeof(_app_info) ? sizeof(_app_info) : app_info_len);
    memcpy(_app_info, &cmd_frame[8], _app_info_len);
#ifdef WITH_COMPANION_FRAME_FRAGMENTS
    resetFragmentSession();
    _app_supports_frmfrg1 = companion_app::hasCapability(&cmd_frame[8], len - 8, "frmfrg1");
#else
    resetAppSyncState();
#endif
    MESH_DEBUG_PRINTLN("App %s connected", app_name);
    _app_supports_mctxt = companion_app::hasCapability(&cmd_frame[8], len - 8, "mctxt");
    _app_supports_mcmp = companion_app::hasCapability(&cmd_frame[8], len - 8, "mcmp");
    _app_supports_aeic = companion_app::hasCapability(&cmd_frame[8], len - 8, "aeic") ||
                         companion_app::appNameIsMeshCoreOpen(&cmd_frame[8], len - 8);
    _app_supports_mcoimg = companion_app::hasCapability(&cmd_frame[8], len - 8, "mcimg");
#ifdef WITH_COMPANION_FRAME_FRAGMENTS
    // The two conditions that decide whether a long frame (a route record, for
    // one) can be delivered at all. Logged here because "the message came but
    // its route did not" is otherwise indistinguishable from a node that never
    // heard the frame.
    OQ_LOG("app start: frmfrg1=%d framed_pref=%d\n",
           _app_supports_frmfrg1 ? 1 : 0, _prefs.framed_packets_enabled ? 1 : 0);
#endif
#if WITH_OFFLINE_ROUTES
    // A new app session: routes this node pushed live to an earlier one are owed
    // to this one again (the client forgets them on disconnect).
    app_session = (uint8_t)((app_session + 1) % OFFLINE_LIVE_NONE);
#endif

    _iter_started = false; // stop any left-over ContactsIterator
    int i = 0;
    out_frame[i++] = RESP_CODE_SELF_INFO;
    out_frame[i++] = ADV_TYPE_CHAT; // what this node Advert identifies as (maybe node's pronouns too?? :-)
    out_frame[i++] = _prefs.tx_power_dbm;
    out_frame[i++] = MAX_LORA_TX_POWER;
    memcpy(&out_frame[i], self_id.pub_key, PUB_KEY_SIZE);
    i += PUB_KEY_SIZE;

    int32_t lat, lon;
    lat = (sensors.node_lat * 1000000.0);
    lon = (sensors.node_lon * 1000000.0);
    memcpy(&out_frame[i], &lat, 4);
    i += 4;
    memcpy(&out_frame[i], &lon, 4);
    i += 4;
    out_frame[i++] = _prefs.multi_acks; // new v7+
    out_frame[i++] = _prefs.advert_loc_policy;
    out_frame[i++] = (_prefs.telemetry_mode_env << 4) | (_prefs.telemetry_mode_loc << 2) |
                     (_prefs.telemetry_mode_base); // v5+
    out_frame[i++] = _prefs.manual_add_contacts;

    uint32_t freq = _prefs.freq * 1000;
    memcpy(&out_frame[i], &freq, 4);
    i += 4;
    uint32_t bw = _prefs.bw * 1000;
    memcpy(&out_frame[i], &bw, 4);
    i += 4;
    out_frame[i++] = _prefs.sf;
    out_frame[i++] = _prefs.cr;

    int tlen = strlen(_prefs.node_name); // revisit: UTF_8 ??
    memcpy(&out_frame[i], _prefs.node_name, tlen);
    i += tlen;
    _serial->writeFrame(out_frame, i);
  } else if (cmd_frame[0] == CMD_RUN_CLI_COMMAND && len >= 3) { // V14+
    int i = 1;
    char *text = (char *)&cmd_frame[i];
    int tlen = len - i;
    text[tlen] = 0; // ensure null

    reply_buf[0] = 0;
    if (!handleCommand(text, 0, reply_buf)) {
      strcat(reply_buf, "Unknown command");   // reply_buf may have cmd prefix from 'text'
    }
    out_frame[0] = RESP_CODE_CLI_REPLY;
    int rlen = strlen(reply_buf);
    memcpy(&out_frame[1], reply_buf, rlen);
    _serial->writeFrame(out_frame, 1 + rlen);
  } else if (cmd_frame[0] == CMD_SEND_TXT_MSG && len >= 14) {
    int i = 1;
    uint8_t txt_type = cmd_frame[i++];
    uint8_t attempt = cmd_frame[i++];
    uint32_t msg_timestamp;
    memcpy(&msg_timestamp, &cmd_frame[i], 4);
    i += 4;
    uint8_t *pub_key_prefix = &cmd_frame[i];
    i += 6;
    ContactInfo *recipient = lookupContactByPubKey(pub_key_prefix, 6);
    if (recipient && (txt_type == TXT_TYPE_PLAIN || txt_type == TXT_TYPE_CLI_DATA || txt_type == TXT_TYPE_CLI_COMMAND)) {
      char *text = (char *)&cmd_frame[i];
      int tlen = len - i;
      uint32_t est_timeout;
      text[tlen] = 0; // ensure null
      int result;
      uint32_t expected_ack;
      uint32_t app_ack = 0;
      if (txt_type == TXT_TYPE_CLI_DATA || txt_type == TXT_TYPE_CLI_COMMAND) {
        msg_timestamp = getRTCClock()->getCurrentTimeUnique(); // Use node's RTC instead of app timestamp to avoid tripping replay protection
        result = sendCommandData(*recipient, msg_timestamp, attempt, txt_type, text, est_timeout);
        expected_ack = 0; // no Ack expected
      } else {
        const char* send_text = text;
        char text_buf[MAX_FRAME_SIZE + 1];
        if (_cyr2lat_contacts_enabled) {
          transliterateChannelText(text, tlen, text_buf, sizeof(text_buf));
          send_text = text_buf;
        }
        result = sendMessage(*recipient, msg_timestamp, attempt, send_text, expected_ack, est_timeout);
        // The acknowledgement on the air names the text that went out, while
        // an app that computes the hash itself, rather than taking it from
        // RESP_CODE_SENT, waits for the hash of the text it handed over. When
        // the two texts differ (transformed by transliteration), the app is
        // told, and later confirmed, its own hash.
        if (result != MSG_SEND_FAILED && expected_ack &&
            text_message_ack::textTransformed(text, send_text)) {
          app_ack = text_message_ack::expectedAck(msg_timestamp, attempt, text, self_id.pub_key);
        }
      }
      // TODO: add expected ACK to table
      if (result == MSG_SEND_FAILED) {
        writeErrFrame(ERR_CODE_TABLE_FULL);
      } else {
        if (expected_ack) {
          expected_ack_table[next_ack_idx].msg_sent = _ms->getMillis(); // add to circular table
          expected_ack_table[next_ack_idx].ack = expected_ack;
          expected_ack_table[next_ack_idx].app_ack = app_ack;
          expected_ack_table[next_ack_idx].contact = recipient;
          next_ack_idx = (next_ack_idx + 1) % EXPECTED_ACK_TABLE_SIZE;
        }

        out_frame[0] = RESP_CODE_SENT;
        out_frame[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
        memcpy(&out_frame[2], app_ack ? &app_ack : &expected_ack, 4);
        memcpy(&out_frame[6], &est_timeout, 4);
        _serial->writeFrame(out_frame, 10);
      }
    } else {
      writeErrFrame(recipient == NULL
                        ? ERR_CODE_NOT_FOUND
                        : ERR_CODE_UNSUPPORTED_CMD); // unknown recipient, or unsupported TXT_TYPE_*
    }
  } else if (cmd_frame[0] == CMD_SEND_CHANNEL_TXT_MSG) { // send GroupChannel text msg
    int i = 1;
    uint8_t txt_type = cmd_frame[i++]; // should be TXT_TYPE_PLAIN
    uint8_t channel_idx = cmd_frame[i++];
    uint32_t msg_timestamp;
    memcpy(&msg_timestamp, &cmd_frame[i], 4);
    i += 4;
    const char *text = (char *)&cmd_frame[i];

    if (txt_type != TXT_TYPE_PLAIN) {
      writeErrFrame(ERR_CODE_UNSUPPORTED_CMD);
    } else {
      ChannelDetails channel;
      bool success = getChannel(channel_idx, channel);
      if (success && strcmp(channel.name, "TerminalCLI") == 0) {
#ifdef WITH_COMPANION_CLI
        const_cast<char*>(text)[len - i] = '\0'; // text is not null-terminated in this frame type
        // Acknowledge acceptance BEFORE running the command. Apps clear their
        // composer on this response, while the CLI answer is a separate queued
        // message, so anything slow in between (chat-log flash write, a
        // scheduled reboot that drops the link) made a command the node did
        // execute look like a failed send. flushSend() is what makes the ack
        // early on the queueing transports: writeFrame() only enqueues, and the
        // queue is otherwise pumped once per loop - after the command.
        writeOKFrame();
        _serial->flushSend();
        if (_listener) _listener->onMessageSent(channel.name, text, false, false);
        handleTerminalCLI(channel_idx, msg_timestamp, text, true);
#else
        writeErrFrame(ERR_CODE_UNSUPPORTED_CMD);
#endif
      } else if (success) {
        const char* send_text = text;
        int send_len = len - i;
        int orig_len = send_len;
        char text_buf[MAX_FRAME_SIZE + 1];
        bool transformed = false;

        if (_cyr2lat_channels_enabled) {
          send_len = transliterateChannelText(text, send_len, text_buf, sizeof(text_buf));
          send_text = text_buf;
          transformed = textDiffers(text, orig_len, send_text, send_len);
        }
        int prefix_len = strlen(_prefs.node_name) + 2; // "<sender>: "
        if (send_len + prefix_len > MAX_TEXT_LEN) {
          send_len = MAX_TEXT_LEN - prefix_len;
          if (send_len < 0) send_len = 0;
        }
        if (send_text != text_buf) {
          int copy_n = send_len < (int)sizeof(text_buf) - 1 ? send_len : (int)sizeof(text_buf) - 1;
          memcpy(text_buf, send_text, copy_n);
          send_text = text_buf;
          send_len  = copy_n;
        }
        text_buf[send_len] = '\0';
        if (sendGroupMessageWithCyr2LatMap(msg_timestamp, channel.channel, _prefs.node_name, send_text, send_len,
                                           text, orig_len, transformed)) {
          if (_listener) _listener->onMessageSent(channel.name, send_text, false, false);
          writeOKFrame();
        } else {
          writeErrFrame(ERR_CODE_NOT_FOUND);
        }
      } else {
        writeErrFrame(ERR_CODE_NOT_FOUND); // bad channel_idx
      }
    }
  } else if (cmd_frame[0] == CMD_SEND_CHANNEL_DATA) { // send GroupChannel datagram
    if (len < 4) {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
      return;
    }
    int i = 1;
    uint8_t channel_idx = cmd_frame[i++];
    uint8_t path_len = cmd_frame[i++];

    // validate path len, allowing 0xFF for flood
    if (!mesh::Packet::isValidPathLen(path_len) && path_len != OUT_PATH_UNKNOWN) {
      MESH_DEBUG_PRINTLN("CMD_SEND_CHANNEL_DATA invalid path size: %d", path_len);
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
      return;
    }

    // parse provided path if not flood
    uint8_t path[MAX_PATH_SIZE];
    if (path_len != OUT_PATH_UNKNOWN) {
      i += mesh::Packet::writePath(path, &cmd_frame[i], path_len);
    }

    uint16_t data_type = ((uint16_t)cmd_frame[i]) | (((uint16_t)cmd_frame[i + 1]) << 8);
    i += 2;
    const uint8_t *payload = &cmd_frame[i];
    int payload_len = (len > (size_t)i) ? (int)(len - i) : 0;

    ChannelDetails channel;
    if (!getChannel(channel_idx, channel)) {
      writeErrFrame(ERR_CODE_NOT_FOUND); // bad channel_idx
    } else if (data_type == DATA_TYPE_RESERVED) {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    } else if (payload_len > MAX_GROUP_DATA_LENGTH) {
      MESH_DEBUG_PRINTLN("CMD_SEND_CHANNEL_DATA payload too long: %d > %d", payload_len, MAX_GROUP_DATA_LENGTH);
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    } else if (sendGroupData(channel.channel, path, path_len, data_type, payload, payload_len)) {
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_TABLE_FULL);
    }
  } else if (cmd_frame[0] == CMD_GET_CONTACTS) { // get Contact list
    if (_iter_started) {
      writeErrFrame(ERR_CODE_BAD_STATE); // iterator is currently busy
    } else {
      if (len >= 5) { // has optional 'since' param
        memcpy(&_iter_filter_since, &cmd_frame[1], 4);
        // App may hold a 'since' stamped by a runaway clock (e.g. 2099): nothing would ever be newer,
        // so new/updated contacts would never reach the app. Resync everything in that case.
        if (_iter_filter_since > getRTCClock()->getCurrentTime()) _iter_filter_since = 0;
      } else {
        _iter_filter_since = 0;
      }

      uint8_t reply[5];
      reply[0] = RESP_CODE_CONTACTS_START;
      uint32_t count = getNumContacts(); // total, NOT filtered count
      memcpy(&reply[1], &count, 4);
      _serial->writeFrame(reply, 5);

      // start iterator
      _iter = startContactsIterator();
      _iter_started = true;
      _most_recent_lastmod = 0;
    }
  } else if (cmd_frame[0] == CMD_SET_ADVERT_NAME && len >= 2) {
    int nlen = len - 1;
    if (nlen > sizeof(_prefs.node_name) - 1) nlen = sizeof(_prefs.node_name) - 1; // max len
    memcpy(_prefs.node_name, &cmd_frame[1], nlen);
    _prefs.node_name[nlen] = 0; // null terminator
    dirty_prefs_expiry = futureMillis(LAZY_PREFS_WRITE_DELAY);
    writeOKFrame();
  } else if (cmd_frame[0] == CMD_SET_ADVERT_LATLON && len >= 9) {
    int32_t lat, lon, alt = 0;
    memcpy(&lat, &cmd_frame[1], 4);
    memcpy(&lon, &cmd_frame[5], 4);
    if (len >= 13) {
      memcpy(&alt, &cmd_frame[9], 4); // for FUTURE support
    }
    if (lat <= 90 * 1E6 && lat >= -90 * 1E6 && lon <= 180 * 1E6 && lon >= -180 * 1E6) {
      sensors.node_lat = ((double)lat) / 1000000.0;
      sensors.node_lon = ((double)lon) / 1000000.0;
      dirty_prefs_expiry = futureMillis(LAZY_PREFS_WRITE_DELAY);
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG); // invalid geo coordinate
    }
  } else if (cmd_frame[0] == CMD_GET_DEVICE_TIME) {
    uint8_t reply[5];
    reply[0] = RESP_CODE_CURR_TIME;
    uint32_t now = getRTCClock()->getCurrentTime();
    memcpy(&reply[1], &now, 4);
    _serial->writeFrame(reply, 5);
  } else if (cmd_frame[0] == CMD_SET_DEVICE_TIME && len >= 5) {
    uint32_t secs;
    memcpy(&secs, &cmd_frame[1], 4);
    uint32_t curr = getRTCClock()->getCurrentTime();
    // The device may have run far ahead (corrupt RTC): then the phone's time is accepted backwards
    // too, and contact lastmod is pulled back with it so the bad time can't return after a reboot.
    bool runaway = secs >= RTC_TIME_SANE_MIN && secs < curr && curr - secs > 3600;
    if (secs >= RTC_TIME_SANE_MAX) {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);   // never accept an implausible far-future time
    } else if (secs >= curr || runaway) {
      getRTCClock()->setCurrentTime(secs);
      if (runaway && clampContactsLastmod(secs) > 0) {
        dirty_contacts_expiry = futureMillis(LAZY_CONTACTS_WRITE_DELAY);
      }
      _ts.clock_restored = false;   // the app's time is a real one
      _app_time_lock_until = secs + APP_TIME_HOLDOFF_SECS;
      noteTimeSource(TIME_SOURCE_APP);
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    }
  } else if (cmd_frame[0] == CMD_SEND_SELF_ADVERT) {
    mesh::Packet* pkt;
    if (_prefs.advert_loc_policy == ADVERT_LOC_NONE) {
      pkt = createSelfAdvert(_prefs.node_name);
    } else {
      pkt = createSelfAdvert(_prefs.node_name, sensors.node_lat, sensors.node_lon);
    }
    if (pkt) {
      if (len >= 2 && cmd_frame[1] == 1) { // optional param (1 = flood, 0 = zero hop)
        unsigned long delay_millis = 0;
        TransportKey default_scope;
        memcpy(&default_scope.key, _prefs.default_scope_key, sizeof(default_scope.key));
        sendFloodScoped(default_scope, pkt, delay_millis);
      } else {
        sendZeroHop(pkt);
      }
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_TABLE_FULL);
    }
  } else if (cmd_frame[0] == CMD_RESET_PATH && len >= 1 + 32) {
    uint8_t *pub_key = &cmd_frame[1];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    if (recipient) {
      recipient->out_path_len = OUT_PATH_UNKNOWN;
      // recipient->lastmod = ??   shouldn't be needed, app already has this version of contact
      dirty_contacts_expiry = futureMillis(LAZY_CONTACTS_WRITE_DELAY);
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND); // unknown contact
    }
  } else if (cmd_frame[0] == CMD_ADD_UPDATE_CONTACT && len >= 1 + 32 + 2 + 1) {
    uint8_t *pub_key = &cmd_frame[1];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    uint32_t last_mod = getRTCClock()->getCurrentTime();  // fallback value if not present in cmd_frame
    if (recipient) {
      updateContactFromFrame(*recipient, last_mod, cmd_frame, len);
      recipient->lastmod = last_mod;
      dirty_contacts_expiry = futureMillis(LAZY_CONTACTS_WRITE_DELAY);
      writeOKFrame();
    } else {
      ContactInfo contact;
      updateContactFromFrame(contact, last_mod, cmd_frame, len);
      contact.lastmod = last_mod;
      contact.sync_since = 0;
      if (addContact(contact)) {
        dirty_contacts_expiry = futureMillis(LAZY_CONTACTS_WRITE_DELAY);
        writeOKFrame();
      } else {
        writeErrFrame(ERR_CODE_TABLE_FULL);
      }
    }
  } else if (cmd_frame[0] == CMD_REMOVE_CONTACT) {
    uint8_t *pub_key = &cmd_frame[1];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    if (recipient && removeContact(*recipient)) {
      _store->deleteBlobByKey(pub_key, PUB_KEY_SIZE);
      dirty_contacts_expiry = futureMillis(LAZY_CONTACTS_WRITE_DELAY);
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND); // not found, or unable to remove
    }
  } else if (cmd_frame[0] == CMD_SHARE_CONTACT) {
    uint8_t *pub_key = &cmd_frame[1];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    if (recipient) {
      if (shareContactZeroHop(*recipient)) {
        writeOKFrame();
      } else {
        writeErrFrame(ERR_CODE_TABLE_FULL); // unable to send
      }
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND);
    }
  } else if (cmd_frame[0] == CMD_GET_CONTACT_BY_KEY) {
    uint8_t *pub_key = &cmd_frame[1];
    ContactInfo *contact = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    if (contact) {
      writeContactRespFrame(RESP_CODE_CONTACT, *contact);
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND); // not found
    }
  } else if (cmd_frame[0] == CMD_EXPORT_CONTACT) {
    if (len < 1 + PUB_KEY_SIZE) {
      // export SELF
      mesh::Packet* pkt;
      if (_prefs.advert_loc_policy == ADVERT_LOC_NONE) {
        pkt = createSelfAdvert(_prefs.node_name);
      } else {
        pkt = createSelfAdvert(_prefs.node_name, sensors.node_lat, sensors.node_lon);
      }
      if (pkt) {
        pkt->header |= ROUTE_TYPE_FLOOD; // would normally be sent in this mode

        out_frame[0] = RESP_CODE_EXPORT_CONTACT;
        uint8_t out_len = pkt->writeTo(&out_frame[1]);
        releasePacket(pkt); // undo the obtainNewPacket()
        _serial->writeFrame(out_frame, out_len + 1);
      } else {
        writeErrFrame(ERR_CODE_TABLE_FULL); // Error
      }
    } else {
      uint8_t *pub_key = &cmd_frame[1];
      ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
      uint8_t out_len;
      if (recipient && (out_len = exportContact(*recipient, &out_frame[1])) > 0) {
        out_frame[0] = RESP_CODE_EXPORT_CONTACT;
        _serial->writeFrame(out_frame, out_len + 1);
      } else {
        writeErrFrame(ERR_CODE_NOT_FOUND); // not found
      }
    }
  } else if (cmd_frame[0] == CMD_IMPORT_CONTACT && len > 2 + 32 + 64) {
    if (importContact(&cmd_frame[1], len - 1)) {
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    }
  } else if (cmd_frame[0] == CMD_SYNC_NEXT_MESSAGE) {
#ifdef WITH_COMPANION_FRAME_FRAGMENTS
    // Parse the ACK before anything else: a stale but well-formed ACK must be
    // ignored rather than turned into an error, and a malformed one rejected.
    const uint8_t flags = len >= 2 ? cmd_frame[1] : 0;
    if (flags & ~SYNC_FLAG_FRAGMENT_ACK) {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
      return;
    }
    const bool has_fragment_ack = (flags & SYNC_FLAG_FRAGMENT_ACK) != 0;
    if (has_fragment_ack && len < 5) {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
      return;
    }
    const uint16_t ack_fragment_id = has_fragment_ack
        ? (uint16_t)cmd_frame[2] | ((uint16_t)cmd_frame[3] << 8) : 0;
    const uint8_t ack_fragment_index = has_fragment_ack ? cmd_frame[4] : 0;
    int out_len = getNextAppFrame(out_frame, has_fragment_ack, ack_fragment_id, ack_fragment_index);
#else
    int out_len = getNextAppFrame(out_frame);
#endif
    if (out_len > 0) {
      if (app_target_ver < 3) {
        const int legacy_len = convertV3FrameToLegacy(out_frame, out_len, out_frame);
        if (legacy_len > 0) out_len = legacy_len;
      }
      _serial->writeFrame(out_frame, out_len);
      if (_listener) _listener->onQueueSizeChanged(offline_queue_len);
    } else {
      out_frame[0] = RESP_CODE_NO_MORE_MESSAGES;
      _serial->writeFrame(out_frame, 1);
    }
  } else if (cmd_frame[0] == CMD_SET_RADIO_PARAMS) {
    int i = 1;
    uint32_t freq;
    memcpy(&freq, &cmd_frame[i], 4);
    i += 4;
    uint32_t bw;
    memcpy(&bw, &cmd_frame[i], 4);
    i += 4;
    uint8_t sf = cmd_frame[i++];
    uint8_t cr = cmd_frame[i++];
    uint8_t repeat = 0;  // default - false
    if (len > i) {
      repeat = cmd_frame[i++];   // FIRMWARE_VER_CODE  9+
    }

    if (repeat && !isValidClientRepeatFreq(freq)) {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    } else if (freq >= 150000 && freq <= 2500000 && sf >= 5 && sf <= 12 && cr >= 5 && cr <= 8 && bw >= 7000 &&
        bw <= 500000) {
      _prefs.sf = sf;
      _prefs.cr = cr;
      _prefs.freq = (float)freq / 1000.0;
      _prefs.bw = (float)bw / 1000.0;
      _prefs.setRepeatEn(repeat != 0);
      savePrefs();

      radio_driver.setParams(_prefs.freq, _prefs.bw, _prefs.sf, _prefs.cr);
      MESH_DEBUG_PRINTLN("OK: CMD_SET_RADIO_PARAMS: f=%d, bw=%d, sf=%d, cr=%d", freq, bw, (uint32_t)sf,
                         (uint32_t)cr);

      writeOKFrame();
    } else {
      MESH_DEBUG_PRINTLN("Error: CMD_SET_RADIO_PARAMS: f=%d, bw=%d, sf=%d, cr=%d", freq, bw, (uint32_t)sf,
                         (uint32_t)cr);
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    }
  } else if (cmd_frame[0] == CMD_SET_RADIO_TX_POWER) {
    int8_t power = (int8_t)cmd_frame[1];
    if (power < -9 || power > MAX_LORA_TX_POWER) {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    } else {
      _prefs.tx_power_dbm = power;
      dirty_prefs_expiry = futureMillis(LAZY_PREFS_WRITE_DELAY);
      radio_driver.setTxPower(_prefs.tx_power_dbm);
      writeOKFrame();
    }
  } else if (cmd_frame[0] == CMD_SET_TUNING_PARAMS) {
    int i = 1;
    uint32_t rx, af;
    memcpy(&rx, &cmd_frame[i], 4);
    i += 4;
    memcpy(&af, &cmd_frame[i], 4);
    i += 4;
    _prefs.rx_delay_base = ((float)rx) / 1000.0f;
    _prefs.airtime_factor = ((float)af) / 1000.0f;
    dirty_prefs_expiry = futureMillis(LAZY_PREFS_WRITE_DELAY);
    writeOKFrame();
  } else if (cmd_frame[0] == CMD_GET_TUNING_PARAMS) {
    uint32_t rx = _prefs.rx_delay_base * 1000, af = _prefs.airtime_factor * 1000;
    int i = 0;
    out_frame[i++] = RESP_CODE_TUNING_PARAMS;
    memcpy(&out_frame[i], &rx, 4); i += 4;
    memcpy(&out_frame[i], &af, 4); i += 4;
    _serial->writeFrame(out_frame, i);
  } else if (cmd_frame[0] == CMD_SET_OTHER_PARAMS) {
    _prefs.manual_add_contacts = cmd_frame[1];
    if (len >= 3) {
      _prefs.telemetry_mode_base = cmd_frame[2] & 0x03; // v5+
      _prefs.telemetry_mode_loc = (cmd_frame[2] >> 2) & 0x03;
      _prefs.telemetry_mode_env = (cmd_frame[2] >> 4) & 0x03;

      if (len >= 4) {
        _prefs.advert_loc_policy = cmd_frame[3];
        if (len >= 5) {
          _prefs.multi_acks = cmd_frame[4];
        }
      }
    }
    dirty_prefs_expiry = futureMillis(LAZY_PREFS_WRITE_DELAY);
    writeOKFrame();
  } else if (cmd_frame[0] == CMD_SET_PATH_HASH_MODE && cmd_frame[1] == 0 && len >= 3) {
    if (cmd_frame[2] >= 3) {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    } else {
      _prefs.path_hash_mode = cmd_frame[2];
      dirty_prefs_expiry = futureMillis(LAZY_PREFS_WRITE_DELAY);
      writeOKFrame();
    }
  } else if (cmd_frame[0] == CMD_REBOOT && memcmp(&cmd_frame[1], "reboot", 6) == 0) {
#ifdef NRF52_PLATFORM
    if (_serial && _serial->isConnected()) {
      // NRF52: can't write flash while BLE connected — defer, loop() will disconnect + save
      _pending_reboot_at = futureMillis(1500);
      _pending_reboot_deadline = futureMillis(30000);
      _app_drain_until = 0;   // fresh command: open a new drain window
    } else {
#else
    {
#endif
      if (dirty_contacts_expiry) { saveContacts(); dirty_contacts_expiry = 0; }
      if (dirty_prefs_expiry)    { savePrefs();    dirty_prefs_expiry    = 0; }
#ifdef ESP32
      SPIFFS.end();
#endif
      board.reboot();
    }
  } else if (cmd_frame[0] == CMD_GET_BATT_AND_STORAGE) {
    uint8_t reply[11];
    int i = 0;
    reply[i++] = RESP_CODE_BATT_AND_STORAGE;
    uint16_t battery_millivolts = board.getBattMilliVolts();
    uint32_t used = _store->getStorageUsedKb();
    uint32_t total = _store->getStorageTotalKb();
    memcpy(&reply[i], &battery_millivolts, 2); i += 2;
    memcpy(&reply[i], &used, 4); i += 4;
    memcpy(&reply[i], &total, 4); i += 4;
    _serial->writeFrame(reply, i);
  } else if (cmd_frame[0] == CMD_EXPORT_PRIVATE_KEY) {
#if ENABLE_PRIVATE_KEY_EXPORT
    uint8_t reply[65];
    reply[0] = RESP_CODE_PRIVATE_KEY;
    self_id.writeTo(&reply[1], 64);
    _serial->writeFrame(reply, 65);
#else
    writeDisabledFrame();
#endif
  } else if (cmd_frame[0] == CMD_IMPORT_PRIVATE_KEY && len >= 65) {
#if ENABLE_PRIVATE_KEY_IMPORT
    if (!mesh::LocalIdentity::validatePrivateKey(&cmd_frame[1])) {
        writeErrFrame(ERR_CODE_ILLEGAL_ARG); // invalid key
    } else {
        mesh::LocalIdentity identity;
        identity.readFrom(&cmd_frame[1], 64);
        if (_store->saveMainIdentity(identity)) {
          self_id = identity;
          writeOKFrame();
          // re-load contacts, to invalidate ecdh shared_secrets
          resetContacts();
          _store->loadContacts(this);
        } else {
          writeErrFrame(ERR_CODE_FILE_IO_ERROR);
        }
    }
#else
    writeDisabledFrame();
#endif
  } else if (cmd_frame[0] == CMD_SEND_RAW_DATA && len >= 6) {
    int i = 1;
    uint8_t path_len = cmd_frame[i++];
    if (path_len >= 0 && mesh::Packet::isValidPathLen(path_len)) {
      uint8_t path[MAX_PATH_SIZE];
      i += mesh::Packet::writePath(path, &cmd_frame[i], path_len);
      if (i + 4 > len) {  // min payload 4 bytes
        writeErrFrame(ERR_CODE_ILLEGAL_ARG);
      } else {
        auto pkt = createRawData(&cmd_frame[i], len - i);
        if (pkt) {
          sendDirect(pkt, path, path_len);
          writeOKFrame();
        } else {
          writeErrFrame(ERR_CODE_TABLE_FULL);
        }
      }
    } else {
      writeErrFrame(ERR_CODE_UNSUPPORTED_CMD); // flood, not supported (yet)
    }
  } else if (cmd_frame[0] == CMD_SEND_LOGIN && len >= 1 + PUB_KEY_SIZE) {
    uint8_t *pub_key = &cmd_frame[1];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    char *password = (char *)&cmd_frame[1 + PUB_KEY_SIZE];
    cmd_frame[len] = 0; // ensure null terminator in password
    if (recipient) {
      uint32_t est_timeout;
      int result = sendLogin(*recipient, password, est_timeout);
      if (result == MSG_SEND_FAILED) {
        writeErrFrame(ERR_CODE_TABLE_FULL);
      } else {
        clearPendingReqs();
        memcpy(&pending_login, recipient->id.pub_key, 4); // match this to onContactResponse()
        out_frame[0] = RESP_CODE_SENT;
        out_frame[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
        memcpy(&out_frame[2], &pending_login, 4);
        memcpy(&out_frame[6], &est_timeout, 4);
        _serial->writeFrame(out_frame, 10);
      }
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND); // contact not found
    }
  } else if (cmd_frame[0] == CMD_SEND_ANON_REQ && len > 1 + PUB_KEY_SIZE) {
    uint8_t *pub_key = &cmd_frame[1];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    ContactInfo anon;
    if (recipient == NULL) { // FIRMWARE_VER_CODE 13+,  allow non-contact requests
      memset(&anon, 0, sizeof(anon));
      memcpy(anon.id.pub_key, pub_key, PUB_KEY_SIZE);
      anon.out_path_len = 0;   // default to zero-hop direct
      anon.type = ADV_TYPE_NONE;  // unknown
      anon.lastmod = getRTCClock()->getCurrentTime();

      if (addContact(anon)) recipient = &anon;
    }
    uint8_t *data = &cmd_frame[1 + PUB_KEY_SIZE];
    if (recipient) {
      uint32_t tag, est_timeout;
      int result = sendAnonReq(*recipient, data, len - (1 + PUB_KEY_SIZE), tag, est_timeout);
      if (result == MSG_SEND_FAILED) {
        writeErrFrame(ERR_CODE_TABLE_FULL);
      } else {
        clearPendingReqs();
        pending_req = tag; // match this to onContactResponse()
        out_frame[0] = RESP_CODE_SENT;
        out_frame[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
        memcpy(&out_frame[2], &tag, 4);
        memcpy(&out_frame[6], &est_timeout, 4);
        _serial->writeFrame(out_frame, 10);
      }
    } else {
      writeErrFrame(ERR_CODE_TABLE_FULL); // contacts full
    }
  } else if (cmd_frame[0] == CMD_SEND_STATUS_REQ && len >= 1 + PUB_KEY_SIZE) {
    uint8_t *pub_key = &cmd_frame[1];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    if (recipient) {
      uint32_t tag, est_timeout;
      int result = sendRequest(*recipient, REQ_TYPE_GET_STATUS, tag, est_timeout);
      if (result == MSG_SEND_FAILED) {
        writeErrFrame(ERR_CODE_TABLE_FULL);
      } else {
        clearPendingReqs();
        // FUTURE:  pending_status = tag;  // match this in onContactResponse()
        memcpy(&pending_status, recipient->id.pub_key, 4); // legacy matching scheme
        out_frame[0] = RESP_CODE_SENT;
        out_frame[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
        memcpy(&out_frame[2], &tag, 4);
        memcpy(&out_frame[6], &est_timeout, 4);
        _serial->writeFrame(out_frame, 10);
      }
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND); // contact not found
    }
  } else if (cmd_frame[0] == CMD_SEND_PATH_DISCOVERY_REQ && cmd_frame[1] == 0 && len >= 2 + PUB_KEY_SIZE) {
    uint8_t *pub_key = &cmd_frame[2];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    if (recipient) {
      uint32_t tag, est_timeout;
      // 'Path Discovery' is just a special case of flood + Telemetry req
      uint8_t req_data[9];
      req_data[0] = REQ_TYPE_GET_TELEMETRY_DATA;
      req_data[1] = ~(TELEM_PERM_BASE);  // NEW: inverse permissions mask (ie. we only want BASE telemetry)
      memset(&req_data[2], 0, 3);  // reserved
      getRNG()->random(&req_data[5], 4);   // random blob to help make packet-hash unique
      auto save = recipient->out_path_len;    // temporarily force sendRequest() to flood
      recipient->out_path_len = OUT_PATH_UNKNOWN;
      int result = sendRequest(*recipient, req_data, sizeof(req_data), tag, est_timeout);
      recipient->out_path_len = save;
      if (result == MSG_SEND_FAILED) {
        writeErrFrame(ERR_CODE_TABLE_FULL);
      } else {
        clearPendingReqs();
        pending_discovery = tag; // match this in onContactResponse()
        out_frame[0] = RESP_CODE_SENT;
        out_frame[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
        memcpy(&out_frame[2], &tag, 4);
        memcpy(&out_frame[6], &est_timeout, 4);
        _serial->writeFrame(out_frame, 10);
      }
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND); // contact not found
    }
  } else if (cmd_frame[0] == CMD_SEND_TELEMETRY_REQ && len >= 4 + PUB_KEY_SIZE) {  // can deprecate, in favour of CMD_SEND_BINARY_REQ
    uint8_t *pub_key = &cmd_frame[4];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    if (recipient) {
      uint32_t tag, est_timeout;
      int result = sendRequest(*recipient, REQ_TYPE_GET_TELEMETRY_DATA, tag, est_timeout);
      if (result == MSG_SEND_FAILED) {
        writeErrFrame(ERR_CODE_TABLE_FULL);
      } else {
        clearPendingReqs();
        pending_telemetry = tag; // match this in onContactResponse()
        out_frame[0] = RESP_CODE_SENT;
        out_frame[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
        memcpy(&out_frame[2], &tag, 4);
        memcpy(&out_frame[6], &est_timeout, 4);
        _serial->writeFrame(out_frame, 10);
      }
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND); // contact not found
    }
  } else if (cmd_frame[0] == CMD_SEND_TELEMETRY_REQ && len == 4) {  // 'self' telemetry request
    telemetry.reset();
    telemetry.addVoltage(TELEM_CHANNEL_SELF, (float)board.getBattMilliVolts() / 1000.0f);
    float temperature = board.getMCUTemperature();
    if(!isnan(temperature)) { // Supported boards with built-in temperature sensor. ESP32-C3 may return NAN
      telemetry.addTemperature(TELEM_CHANNEL_SELF, temperature); // Built-in MCU Temperature
    }

    // query other sensors -- target specific
    sensors.querySensors(0xFF, telemetry);

    int i = 0;
    out_frame[i++] = PUSH_CODE_TELEMETRY_RESPONSE;
    out_frame[i++] = 0; // reserved
    memcpy(&out_frame[i], self_id.pub_key, 6);
    i += 6; // pub_key_prefix
    uint8_t tlen = telemetry.getSize();
    memcpy(&out_frame[i], telemetry.getBuffer(), tlen);
    i += tlen;
    _serial->writeFrame(out_frame, i);
  } else if (cmd_frame[0] == CMD_SEND_BINARY_REQ && len >= 2 + PUB_KEY_SIZE) {
    uint8_t *pub_key = &cmd_frame[1];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    if (recipient) {
      uint8_t *req_data = &cmd_frame[1 + PUB_KEY_SIZE];
      uint32_t tag, est_timeout;
      int result = sendRequest(*recipient, req_data, len - (1 + PUB_KEY_SIZE), tag, est_timeout);
      if (result == MSG_SEND_FAILED) {
        writeErrFrame(ERR_CODE_TABLE_FULL);
      } else {
        clearPendingReqs();
        pending_req = tag; // match this in onContactResponse()
        out_frame[0] = RESP_CODE_SENT;
        out_frame[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
        memcpy(&out_frame[2], &tag, 4);
        memcpy(&out_frame[6], &est_timeout, 4);
        _serial->writeFrame(out_frame, 10);
      }
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND); // contact not found
    }
  } else if (cmd_frame[0] == CMD_HAS_CONNECTION && len >= 1 + PUB_KEY_SIZE) {
    uint8_t *pub_key = &cmd_frame[1];
    if (hasConnectionTo(pub_key)) {
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND);
    }
  } else if (cmd_frame[0] == CMD_LOGOUT && len >= 1 + PUB_KEY_SIZE) {
    uint8_t *pub_key = &cmd_frame[1];
    stopConnection(pub_key);
    writeOKFrame();
  } else if (cmd_frame[0] == CMD_GET_CHANNEL && len >= 2) {
    uint8_t channel_idx = cmd_frame[1];
    ChannelDetails channel;
    if (getChannel(channel_idx, channel)) {
      int i = 0;
      out_frame[i++] = RESP_CODE_CHANNEL_INFO;
      out_frame[i++] = channel_idx;
      strcpy((char *)&out_frame[i], channel.name);
      i += 32;
      memcpy(&out_frame[i], channel.channel.secret, 16);
      i += 16; // NOTE: only 128-bit supported
      _serial->writeFrame(out_frame, i);
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND);
    }
  } else if (cmd_frame[0] == CMD_SET_CHANNEL && len >= 2 + 32 + 32) {
    writeErrFrame(ERR_CODE_UNSUPPORTED_CMD); // not supported (yet)
  } else if (cmd_frame[0] == CMD_SET_CHANNEL && len >= 2 + 32 + 16) {
    uint8_t channel_idx = cmd_frame[1];
    ChannelDetails channel;
    StrHelper::strncpy(channel.name, (char *)&cmd_frame[2], 32);
    memset(channel.channel.secret, 0, sizeof(channel.channel.secret));
    memcpy(channel.channel.secret, &cmd_frame[2 + 32], 16); // NOTE: only 128-bit supported
    if (setChannel(channel_idx, channel)) {
      saveChannels();
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND); // bad channel_idx
    }
  } else if (cmd_frame[0] == CMD_SIGN_START) {
    out_frame[0] = RESP_CODE_SIGN_START;
    out_frame[1] = 0; // reserved
    uint32_t len = MAX_SIGN_DATA_LEN;
    memcpy(&out_frame[2], &len, 4);
    _serial->writeFrame(out_frame, 6);

    if (sign_data) {
      free(sign_data);
    }
    sign_data = (uint8_t *)malloc(MAX_SIGN_DATA_LEN);
    sign_data_len = 0;
  } else if (cmd_frame[0] == CMD_SIGN_DATA && len > 1) {
    if (sign_data == NULL || sign_data_len + (len - 1) > MAX_SIGN_DATA_LEN) {
      writeErrFrame(sign_data == NULL ? ERR_CODE_BAD_STATE : ERR_CODE_TABLE_FULL); // error: too long
    } else {
      memcpy(&sign_data[sign_data_len], &cmd_frame[1], len - 1);
      sign_data_len += (len - 1);
      writeOKFrame();
    }
  } else if (cmd_frame[0] == CMD_SIGN_FINISH) {
    if (sign_data) {
      self_id.sign(&out_frame[1], sign_data, sign_data_len);

      free(sign_data); // don't need sign_data now
      sign_data = NULL;

      out_frame[0] = RESP_CODE_SIGNATURE;
      _serial->writeFrame(out_frame, 1 + SIGNATURE_SIZE);
    } else {
      writeErrFrame(ERR_CODE_BAD_STATE);
    }
  } else if (cmd_frame[0] == CMD_SEND_TRACE_PATH && len > 10 && len - 10 < MAX_PACKET_PAYLOAD-5) {
    uint8_t path_len = len - 10;
    uint8_t flags = cmd_frame[9];
    uint8_t path_sz = flags & 0x03;  // NEW v1.11+
    if ((path_len >> path_sz) > MAX_PATH_SIZE || (path_len % (1 << path_sz)) != 0) { // make sure is multiple of path_sz
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    } else {
      uint32_t tag, auth;
      memcpy(&tag, &cmd_frame[1], 4);
      memcpy(&auth, &cmd_frame[5], 4);
      auto pkt = createTrace(tag, auth, flags);
      if (pkt) {
        sendDirect(pkt, &cmd_frame[10], path_len);

        uint32_t t = _radio->getEstAirtimeFor(pkt->payload_len + pkt->path_len + 2);
        uint32_t est_timeout = calcDirectTimeoutMillisFor(t, path_len >> path_sz);

        out_frame[0] = RESP_CODE_SENT;
        out_frame[1] = 0;
        memcpy(&out_frame[2], &tag, 4);
        memcpy(&out_frame[6], &est_timeout, 4);
        _serial->writeFrame(out_frame, 10);
      } else {
        writeErrFrame(ERR_CODE_TABLE_FULL);
      }
    }
  } else if (cmd_frame[0] == CMD_SET_DEVICE_PIN && len >= 5) {

    // get pin from command frame
    uint32_t pin;
    memcpy(&pin, &cmd_frame[1], 4);

    // ensure pin is zero, or a valid 6 digit pin
    if (pin == 0 || (pin >= 100000 && pin <= 999999)) {
      _prefs.ble_pin = pin;
      if (pin != 0) _active_ble_pin = pin;  // don't zero out display — pin unchanged until reboot
      dirty_prefs_expiry = futureMillis(LAZY_PREFS_WRITE_DELAY);
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    }
  } else if (cmd_frame[0] == CMD_GET_CUSTOM_VARS) {
    out_frame[0] = RESP_CODE_CUSTOM_VARS;
    char *dp = (char *)&out_frame[1];
    for (int i = 0; i < sensors.getNumSettings() && dp - (char *)&out_frame[1] < 140; i++) {
      if (i > 0) {
        *dp++ = ',';
      }
      strcpy(dp, sensors.getSettingName(i));
      dp = strchr(dp, 0);
      *dp++ = ':';
      strcpy(dp, sensors.getSettingValue(i));
      dp = strchr(dp, 0);
    }
    _serial->writeFrame(out_frame, dp - (char *)out_frame);
  } else if (cmd_frame[0] == CMD_SET_CUSTOM_VAR && len >= 4) {
    cmd_frame[len] = 0;
    char *sp = (char *)&cmd_frame[1];
    char *np = strchr(sp, ':'); // look for separator char
    if (np) {
      *np++ = 0; // modify 'cmd_frame', replace ':' with null
      bool success = sensors.setSettingValue(sp, np);
      if (success) {
        #if ENV_INCLUDE_GPS == 1
        // Update node preferences for GPS settings
        if (strcmp(sp, "gps") == 0) {
          _prefs.gps_enabled = (np[0] == '1') ? 1 : 0;
          dirty_prefs_expiry = futureMillis(LAZY_PREFS_WRITE_DELAY);
        } else if (strcmp(sp, "gps_interval") == 0) {
          uint32_t interval_seconds = atoi(np);
          _prefs.gps_interval = constrain(interval_seconds, 0, 86400);
          dirty_prefs_expiry = futureMillis(LAZY_PREFS_WRITE_DELAY);
        }
        #endif
        writeOKFrame();
      } else {
        writeErrFrame(ERR_CODE_ILLEGAL_ARG);
      }
    } else {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    }
  } else if (cmd_frame[0] == CMD_GET_ADVERT_PATH && len >= PUB_KEY_SIZE+2) {
    // FUTURE use:  uint8_t reserved = cmd_frame[1];
    uint8_t *pub_key = &cmd_frame[2];
    AdvertPath* found = NULL;
    for (int i = 0; i < ADVERT_PATH_TABLE_SIZE; i++) {
      auto p = &advert_paths[i];
      if (memcmp(p->pubkey_prefix, pub_key, sizeof(p->pubkey_prefix)) == 0) {
        found = p;
        break;
      }
    }
    if (found) {
      int i = 0;
      out_frame[i++] = RESP_CODE_ADVERT_PATH;
      memcpy(&out_frame[i], &found->recv_timestamp, 4); i += 4;
      out_frame[i++] = found->path_len;
      i += mesh::Packet::writePath(&out_frame[i], found->path, found->path_len);
      _serial->writeFrame(out_frame, i);
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND);
    }
  } else if (cmd_frame[0] == CMD_GET_STATS && len >= 2) {
    uint8_t stats_type = cmd_frame[1];
    if (stats_type == STATS_TYPE_CORE) {
      int i = 0;
      out_frame[i++] = RESP_CODE_STATS;
      out_frame[i++] = STATS_TYPE_CORE;
      uint16_t battery_mv = board.getBattMilliVolts();
      uint32_t uptime_secs = _ms->getMillis() / 1000;
      uint8_t queue_len = (uint8_t)_mgr->getOutboundTotal();
      memcpy(&out_frame[i], &battery_mv, 2); i += 2;
      memcpy(&out_frame[i], &uptime_secs, 4); i += 4;
      memcpy(&out_frame[i], &_err_flags, 2); i += 2;
      out_frame[i++] = queue_len;
      _serial->writeFrame(out_frame, i);
    } else if (stats_type == STATS_TYPE_RADIO) {
      int i = 0;
      out_frame[i++] = RESP_CODE_STATS;
      out_frame[i++] = STATS_TYPE_RADIO;
      int16_t noise_floor = (int16_t)_radio->getNoiseFloor();
      int8_t last_rssi = (int8_t)radio_driver.getLastRSSI();
      int8_t last_snr = (int8_t)(radio_driver.getLastSNR() * 4); // scaled by 4 for 0.25 dB precision
      uint32_t tx_air_secs = getTotalAirTime() / 1000;
      uint32_t rx_air_secs = getReceiveAirTime() / 1000;
      memcpy(&out_frame[i], &noise_floor, 2); i += 2;
      out_frame[i++] = last_rssi;
      out_frame[i++] = last_snr;
      memcpy(&out_frame[i], &tx_air_secs, 4); i += 4;
      memcpy(&out_frame[i], &rx_air_secs, 4); i += 4;
      _serial->writeFrame(out_frame, i);
    } else if (stats_type == STATS_TYPE_PACKETS) {
      int i = 0;
      out_frame[i++] = RESP_CODE_STATS;
      out_frame[i++] = STATS_TYPE_PACKETS;
      uint32_t recv = radio_driver.getPacketsRecv();
      uint32_t sent = radio_driver.getPacketsSent();
      uint32_t n_sent_flood = getNumSentFlood();
      uint32_t n_sent_direct = getNumSentDirect();
      uint32_t n_recv_flood = getNumRecvFlood();
      uint32_t n_recv_direct = getNumRecvDirect();
      uint32_t n_recv_errors = radio_driver.getPacketsRecvErrors();
      memcpy(&out_frame[i], &recv, 4); i += 4;
      memcpy(&out_frame[i], &sent, 4); i += 4;
      memcpy(&out_frame[i], &n_sent_flood, 4); i += 4;
      memcpy(&out_frame[i], &n_sent_direct, 4); i += 4;
      memcpy(&out_frame[i], &n_recv_flood, 4); i += 4;
      memcpy(&out_frame[i], &n_recv_direct, 4); i += 4;
      memcpy(&out_frame[i], &n_recv_errors, 4); i += 4;
      _serial->writeFrame(out_frame, i);
    } else {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG); // invalid stats sub-type
    }
  } else if (cmd_frame[0] == CMD_FACTORY_RESET && memcmp(&cmd_frame[1], "reset", 5) == 0) {
    if (_serial) {
      MESH_DEBUG_PRINTLN("Factory reset: disabling serial interface to prevent reconnects (BLE/WiFi)");
      _serial->disable(); // Phone app disconnects before we can send OK frame so it's safe here
    }
    bool success = _store->formatFileSystem();
    if (success) {
      writeOKFrame();
      delay(1000);
      board.reboot();  // doesn't return
    } else {
      writeErrFrame(ERR_CODE_FILE_IO_ERROR);
    }
  } else if (cmd_frame[0] == CMD_SET_FLOOD_SCOPE_KEY && len >= 2 && cmd_frame[1] == 0) {
    if (len >= 2 + 16) {
      memcpy(send_scope.key, &cmd_frame[2], sizeof(send_scope.key));  // set scope override TransportKey
    } else {
      memset(send_scope.key, 0, sizeof(send_scope.key));  // reset scope override
    }
    send_unscoped = false;
    writeOKFrame();
  } else if (cmd_frame[0] == CMD_SET_FLOOD_SCOPE_KEY && len >= 2 && cmd_frame[1] == 1) {  // ver 12+
    send_unscoped = true;
    writeOKFrame();
  } else if (cmd_frame[0] == CMD_SET_DEFAULT_FLOOD_SCOPE && len >= 1) {
    if (len >= 1+31+16) {
      int n = strlen((char *) &cmd_frame[1]);
      if (n > 0 && n < 31) {
        strcpy(_prefs.default_scope_name, (char *) &cmd_frame[1]);
        memcpy(_prefs.default_scope_key, &cmd_frame[1+31], 16);
        dirty_prefs_expiry = futureMillis(LAZY_PREFS_WRITE_DELAY);
        writeOKFrame();
      } else {
        writeErrFrame(ERR_CODE_ILLEGAL_ARG);
      }
    } else {
      memset(_prefs.default_scope_name, 0, sizeof(_prefs.default_scope_name));  // set default scope to null
      memset(_prefs.default_scope_key, 0, sizeof(_prefs.default_scope_key));
      dirty_prefs_expiry = futureMillis(LAZY_PREFS_WRITE_DELAY);
      writeOKFrame();
    }
  } else if (cmd_frame[0] == CMD_GET_DEFAULT_FLOOD_SCOPE) {
    out_frame[0] = RESP_CODE_DEFAULT_FLOOD_SCOPE;
    if (strlen(_prefs.default_scope_name) > 0) {
      memcpy(&out_frame[1], _prefs.default_scope_name, 31);
      memcpy(&out_frame[1+31], _prefs.default_scope_key, 16);
      _serial->writeFrame(out_frame, 1+31+16);
    } else {
      _serial->writeFrame(out_frame, 1);   // no name or key means null
    }
  } else if (cmd_frame[0] == CMD_SEND_CONTROL_DATA && len >= 2 && (cmd_frame[1] & 0x80) != 0) {
    auto resp = createControlData(&cmd_frame[1], len - 1);
    if (resp) {
      sendZeroHop(resp);
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_TABLE_FULL);
    }
  } else if (cmd_frame[0] == CMD_SET_AUTOADD_CONFIG) {
    _prefs.autoadd_config = cmd_frame[1];
    if (len >= 3) {
      _prefs.autoadd_max_hops = min(cmd_frame[2], (uint8_t)64);
    }
    dirty_prefs_expiry = futureMillis(LAZY_PREFS_WRITE_DELAY);
    writeOKFrame();
  } else if (cmd_frame[0] == CMD_GET_AUTOADD_CONFIG) {
    int i = 0;
    out_frame[i++] = RESP_CODE_AUTOADD_CONFIG;
    out_frame[i++] = _prefs.autoadd_config;
    out_frame[i++] = _prefs.autoadd_max_hops;
    _serial->writeFrame(out_frame, i);
  } else if (cmd_frame[0] == CMD_GET_ALLOWED_REPEAT_FREQ) {
    int i = 0;
    out_frame[i++] = RESP_ALLOWED_REPEAT_FREQ;
    for (int k = 0; k < sizeof(repeat_freq_ranges)/sizeof(repeat_freq_ranges[0]) && i + 8 < sizeof(out_frame); k++) {
      auto r = &repeat_freq_ranges[k];
      memcpy(&out_frame[i], &r->lower_freq, 4); i += 4;
      memcpy(&out_frame[i], &r->upper_freq, 4); i += 4;
    }
    _serial->writeFrame(out_frame, i);
  } else if (cmd_frame[0] == CMD_SEND_RAW_PACKET && len >= 4) {
    auto pkt = obtainNewPacket();
    if (pkt) {
      uint8_t priority = cmd_frame[1];
      if (tryParsePacket(pkt, &cmd_frame[2], len - 2)) {
        sendPacket(pkt, priority, 0);
        writeOKFrame();
      } else {
        releasePacket(pkt);
        writeErrFrame(ERR_CODE_ILLEGAL_ARG);
      }
    } else {
      writeErrFrame(ERR_CODE_TABLE_FULL);
    }
  } else {
    writeErrFrame(ERR_CODE_UNSUPPORTED_CMD);
    MESH_DEBUG_PRINTLN("ERROR: unknown command: %02X", cmd_frame[0]);
  }
}

static bool save_filter(const ContactInfo& c) {
  return c.type != ADV_TYPE_NONE;   // don't save the transient/anon entries
}

void MyMesh::saveContacts() {
  _store->saveContacts(this, save_filter);
}

void MyMesh::enterCLIRescue() {
#ifdef WITH_COMMS_SWITCHING
  // USB carries the companion protocol on the same Serial the rescue CLI would
  // print its banner and echo to: refuse to enter and leave the link clean.
  // (On ESP32 and nRF52 alike; the app offers its own CLI over USB.)
  if (isUsbComms()) return;
#endif
  _cli_rescue = true;
  cli_command[0] = 0;
  Serial.println("========= CLI Rescue =========");
}

// Rescue mode diverts the loop away from checkSerialInterface(), so the
// companion protocol stops dead while it is on. It used to be one-way — the
// flag was cleared only in the constructor — which turned an accidental
// long-press during the first seconds after boot into a phone app that stayed
// broken until the node was power-cycled, with nothing on screen to say why.
void MyMesh::exitCLIRescue() {
  _cli_rescue = false;
  cli_command[0] = 0;
  Serial.println("========= CLI Rescue OFF =========");
}

bool MyMesh::handleCommand(const char* command, uint32_t sender_timestamp, char* reply) {
  while (*command == ' ') command++; // skip leading spaces

  if (strlen(command) > 4 && command[2] == '|') { // optional prefix (for companion radio CLI)
    memcpy(reply, command, 3);                    // reflect the prefix back
    reply += 3;
    *reply = 0;
    command += 3;
  }

#if defined(WITH_COMMS_SWITCHING) && defined(ENABLE_WIFI_INTERFACE)
  if (memcmp(command, "set wifi.", 9) == 0 && handleUniWifiSet(command, reply)) return true;
#endif
#ifdef WITH_COMMS_SWITCHING
  // Generic transport selection, available on every platform that switches at
  // runtime (the wifi.* commands above are WiFi-only).
  if (strcmp(command, "get comms") == 0) {
    const char* mode_str = _wifi_prefs.comms_mode == COMMS_MODE_USB  ? "USB"
                         : _wifi_prefs.comms_mode == COMMS_MODE_WIFI ? "WiFi" : "BLE";
    snprintf(reply, 512, "> comms: %s", mode_str);
    return true;
  }
  if (memcmp(command, "set comms ", 10) == 0) {
    char arg[8];
    size_t i = 0;
    for (const char* p = command + 10; *p && *p != ' ' && i < sizeof(arg) - 1; p++)
      arg[i++] = (char)tolower((unsigned char)*p);
    arg[i] = 0;
    uint8_t mode = COMMS_MODE_BLE;
    if (strcmp(arg, "usb") == 0)       mode = COMMS_MODE_USB;
    else if (strcmp(arg, "wifi") == 0) mode = COMMS_MODE_WIFI;
    else if (strcmp(arg, "ble") != 0) {
      strcpy(reply, "Usage: set comms <ble|usb|wifi>");
      return true;
    }
    if (!commsModeAvailable(mode)) {
      snprintf(reply, 512, "> comms: %s not available in this build", arg);
      return true;
    }
    switchCommsMode(mode);
    snprintf(reply, 512, "> comms: %s", arg);
    return true;
  }
#endif

  if (_prefs.getRadioPrefs()->handleCommand(command, sender_timestamp, reply)) {  // is radio CLI command?
    if (_prefs.getRadioPrefs()->isDirty()) { savePrefs(); }
    return true;
  }

  // hook for variant-specific CLI processing
  if (board.handleCommand(command, sender_timestamp, reply)) {
    if (_prefs.isDirty()) { savePrefs(); }
    return true;
  }

  if (strcmp(command, "reboot") == 0) {
    board.reboot(); // doesn't return
    return true;
  }

  if (strcmp(command, "poweroff") == 0 || strcmp(command, "shutdown") == 0) {
    board.powerOff(); // doesn't return
    return true;
  }

  if (memcmp(command, "set name ", 9) == 0) {
    if (AdvertDataParser::isValidName(&command[9])) {
      StrHelper::strncpy(_prefs.node_name, &command[9], sizeof(_prefs.node_name));
      savePrefs();
      strcpy(reply, "OK");
    } else {
      strcpy(reply, "Error, bad chars");
    }
    return true;
  }
  if (strcmp(command, "get name") == 0) {
    sprintf(reply, "> %s", _prefs.node_name);
    return true;
  }

  if (memcmp(command, "set pin ", 8) == 0) {
    _prefs.ble_pin = atoi(&command[8]);
    savePrefs();
    sprintf(reply, "> pin is now %06d", _prefs.ble_pin);
    return true;
  }

#ifdef ENABLE_WIFI_INTERFACE
  if (memcmp(command, "set wifi.ssid ", 14) == 0) {
    StrHelper::strncpy(_prefs.wifi_ssid, &command[14], sizeof(_prefs.wifi_ssid));
    savePrefs();
    sprintf(reply, "> wifi.ssid is now %s (set wifi.pwd too, then reboot)", _prefs.wifi_ssid);
    return true;
  }
  if (memcmp(command, "set wifi.pwd ", 13) == 0) {
    StrHelper::strncpy(_prefs.wifi_pwd, &command[13], sizeof(_prefs.wifi_pwd));
    savePrefs();
    strcpy(reply, "> wifi.pwd updated (reboot to apply)");
    return true;
  }
  if (strcmp(command, "get wifi.pwd") == 0) {
    sprintf(reply, "> %s", _prefs.wifi_pwd);
    return true;
  }
  if (strcmp(command, "set wifi.clear") == 0) {
    _prefs.wifi_ssid[0] = 0;
    _prefs.wifi_pwd[0] = 0;
    savePrefs();
    strcpy(reply, "> wifi config cleared (reboot to apply)");
    return true;
  }
  if (strcmp(command, "get wifi.ssid") == 0) {
    sprintf(reply, "> %s", _prefs.getWifiSSID()[0] ? _prefs.getWifiSSID() : "(not set)");
    return true;
  }
  if (memcmp(command, "set wifi.enabled ", 17) == 0) {
    _prefs.wifi_enabled = atoi(&command[17]) ? 1 : 0;
    savePrefs();
    sprintf(reply, "> wifi.enabled is now %d (reboot to apply)", _prefs.wifi_enabled);
    return true;
  }
  if (strcmp(command, "get wifi.enabled") == 0) {
    sprintf(reply, "> %d", _prefs.wifi_enabled);
    return true;
  }
  if (strcmp(command, "get wifi.status") == 0) {
    strcpy(reply, WiFi.status() == WL_CONNECTED ? "> connected" : "> disconnected");
    return true;
  }
  if (strcmp(command, "get wifi.ip") == 0) {
    if (WiFi.status() == WL_CONNECTED) {
      sprintf(reply, "> %s", WiFi.localIP().toString().c_str());
    } else {
      strcpy(reply, "> (not connected)");
    }
    return true;
  }
#endif

  if (strcmp(command, "board") == 0) {
    strcpy(reply, board.getManufacturerName());
    return true;
  }

  if (strcmp(command, "ver") == 0) {
    sprintf(reply, "%s (Build: %s)", FIRMWARE_VERSION, FIRMWARE_BUILD_DATE);
    return true;
  }

  if (strcmp(command, "get tz.offset") == 0) {
    sprintf(reply, "> %d", _prefs.tz_offset);
    return true;
  }
  if (memcmp(command, "set tz.offset ", 14) == 0) {
    int8_t tz = atof(&command[14]);
    if (tz < -12 || tz > 14) {
      strcpy(reply, "Error, must be from -12 to +14");
    } else {
      _prefs.tz_offset = tz;
      savePrefs();
      strcpy(reply, "OK");
    }
    return true;
  }

  return false;  // not handled
}

void MyMesh::checkCLIRescueCmd() {
  int len = strlen(cli_command);
  // `cli_command` must stay NUL-terminated within its bounds. If it ever isn't,
  // strlen() above can return >= sizeof(cli_command) and the loop below would
  // then index past the buffer, so clamp defensively.
  if (len >= (int)sizeof(cli_command)) {
    cli_command[0] = 0;
    len = 0;
  }
  while (Serial.available() && len < sizeof(cli_command)-1) {
    char c = Serial.read();
    if (c != '\n') {
      cli_command[len++] = c;
      cli_command[len] = 0;
    }
    Serial.print(c);  // echo
  }
  if (len == sizeof(cli_command)-1) {  // buffer full: treat as a completed line
    cli_command[sizeof(cli_command)-2] = '\r';  // place end-of-line marker inside the buffer
    cli_command[sizeof(cli_command)-1] = 0;     // keep the buffer NUL-terminated
  }

  if (len > 0 && cli_command[len - 1] == '\r') {  // received complete line
    cli_command[len - 1] = 0;  // replace newline with C string null terminator
    normalizeCliCommand(cli_command);

    reply_buf[0] = 0;
    if (handleCommand(cli_command, 0, reply_buf)) {
      // command was handled, print reply output
      Serial.print("  "); Serial.print(reply_buf); Serial.println();
    } else if (strcmp(cli_command, "rebuild") == 0) {
      bool success = _store->formatFileSystem();
      if (success) {
        _store->saveMainIdentity(self_id);
        savePrefs();
        saveContacts();
        saveChannels();
        Serial.println("  > erase and rebuild done");
      } else {
        Serial.println("  Error: erase failed");
      }
    } else if (strcmp(cli_command, "erase") == 0) {
      bool success = _store->formatFileSystem();
      if (success) {
        Serial.println("  > erase done");
      } else {
        Serial.println("  Error: erase failed");
      }
    } else if (memcmp(cli_command, "ls", 2) == 0) {

      // get path from command e.g: "ls /adafruit"
      const char *path = &cli_command[3];

      bool is_fs2 = false;
      if (memcmp(path, "UserData/", 9) == 0) {
        path += 8; // skip "UserData"
      } else if (memcmp(path, "ExtraFS/", 8) == 0) {
        path += 7; // skip "ExtraFS"
        is_fs2 = true;
      }
      Serial.printf("Listing files in %s\n", path);

      // log each file and directory
      File root = _store->openRead(path);
      if (is_fs2 == false) {
        if (root) {
          File file = root.openNextFile();
          while (file) {
            if (file.isDirectory()) {
              Serial.printf("[dir]  UserData%s/%s\n", path, file.name());
            } else {
              Serial.printf("[file] UserData%s/%s (%d bytes)\n", path, file.name(), file.size());
            }
            // move to next file
            file = root.openNextFile();
          }
          root.close();
        }
      }

      if (is_fs2 == true || strlen(path) == 0 || strcmp(path, "/") == 0) {
        if (_store->getSecondaryFS() != nullptr) {
          File root2 = _store->openRead(_store->getSecondaryFS(), path);
          File file = root2.openNextFile();
          while (file) {
            if (file.isDirectory()) {
              Serial.printf("[dir]  ExtraFS%s/%s\n", path, file.name());
            } else {
              Serial.printf("[file] ExtraFS%s/%s (%d bytes)\n", path, file.name(), file.size());
            }
            // move to next file
            file = root2.openNextFile();
          }
          root2.close();
        }
      }
    } else if (memcmp(cli_command, "cat", 3) == 0) {

      // get path from command e.g: "cat /contacts3"
      const char *path = &cli_command[4];

      bool is_fs2 = false;
      if (memcmp(path, "UserData/", 9) == 0) {
        path += 8; // skip "UserData"
      } else if (memcmp(path, "ExtraFS/", 8) == 0) {
        path += 7; // skip "ExtraFS"
        is_fs2 = true;
      } else {
        Serial.println("Invalid path provided, must start with UserData/ or ExtraFS/");
        cli_command[0] = 0;
        return;
      }

      // log file content as hex
      File file = _store->openRead(path);
      if (is_fs2 == true) {
        file = _store->openRead(_store->getSecondaryFS(), path);
      }
      if(file){

        // get file content
        int file_size = file.available();
        uint8_t buffer[file_size];
        file.read(buffer, file_size);

        // print hex
        mesh::Utils::printHex(Serial, buffer, file_size);
        Serial.print("\n");

        file.close();

      }

    } else if (memcmp(cli_command, "rm ", 3) == 0) {
      // get path from command e.g: "rm /adv_blobs"
      const char *path = &cli_command[3];
      MESH_DEBUG_PRINTLN("Removing file: %s", path);
      // ensure path is not empty, or root dir
      if(!path || strlen(path) == 0 || strcmp(path, "/") == 0){
        Serial.println("Invalid path provided");
      } else {
      bool is_fs2 = false;
      if (memcmp(path, "UserData/", 9) == 0) {
        path += 8; // skip "UserData"
      } else if (memcmp(path, "ExtraFS/", 8) == 0) {
        path += 7; // skip "ExtraFS"
        is_fs2 = true;
      }

        // remove file
        bool removed;
        if (is_fs2) {
          MESH_DEBUG_PRINTLN("Removing file from ExtraFS: %s", path);
          removed = _store->removeFile(_store->getSecondaryFS(), path);
        } else {
          MESH_DEBUG_PRINTLN("Removing file from UserData: %s", path);
          removed = _store->removeFile(path);
        }
        if(removed){
          Serial.println("File removed");
        } else {
          Serial.println("Failed to remove file");
        }

      }

    } else if (strcmp(cli_command, "reboot") == 0) {
      board.reboot();  // doesn't return
    } else if (memcmp(cli_command, "timesync", 8) == 0) {
      char reply[320];
      _ts.buildReply(reply, sizeof(reply), getRTCClock());
      Serial.println(reply);
    } else {
      Serial.println("  Error: unknown command");
    }

    cli_command[0] = 0;  // reset command buffer
  }
}

void MyMesh::checkSerialInterface() {
  size_t len = _serial->checkRecvFrame(cmd_frame);
  if (len > 0) {
    handleCmdFrame(len);
  } else if (_iter_started              // check if our ContactsIterator is 'running'
             && !_serial->isWriteBusy() // don't spam the Serial Interface too quickly!
  ) {
    ContactInfo contact;
    if (_iter.hasNext(this, contact)) {
      if (contact.lastmod > _iter_filter_since) { // apply the 'since' filter
        writeContactRespFrame(RESP_CODE_CONTACT, contact);
        if (contact.lastmod > _most_recent_lastmod) {
          _most_recent_lastmod = contact.lastmod; // save for the RESP_CODE_END_OF_CONTACTS frame
        }
      }
    } else { // EOF
      out_frame[0] = RESP_CODE_END_OF_CONTACTS;
      memcpy(&out_frame[1], &_most_recent_lastmod,
             4); // include the most recent lastmod, so app can update their 'since'
      _serial->writeFrame(out_frame, 5);
      _iter_started = false;
    }
  //} else if (!_serial->isWriteBusy()) {
  //  checkConnections();    // TODO - deprecate the 'Connections' stuff
  }
}

bool MyMesh::hasUndeliveredAppFrames() const {
  if (!_serial || !_serial->isConnected()) return false;
  return _serial->hasPendingSend() || offline_queue_len > 0 || _app_sync_active;
}

bool MyMesh::deferForAppDrain(unsigned long& action_at) {
  // Powering down drops whatever the app has not taken yet: disable() clears the
  // transport send queue and the reboot discards the RAM offline queue. Hold the
  // action back while anything is still outstanding, so a command the node did
  // execute (a CLI reply, an incoming message) is not silently swallowed.
  if (!hasUndeliveredAppFrames()) {
    _app_drain_until = 0;
    return false;
  }
  if (_app_drain_until == 0) _app_drain_until = futureMillis(APP_DRAIN_GRACE_MILLIS);
  if (millisHasNowPassed(_app_drain_until)) {   // app is not collecting; stop waiting
    _app_drain_until = 0;
    return false;
  }
  _serial->flushSend();   // one extra frame per iteration while draining
  action_at = futureMillis(50);
  return true;
}

void MyMesh::loop() {
  BaseChatMesh::loop();

  // An app that disconnects mid-delivery must not carry its multi-part
  // app-sync state into the next app: otherwise the next app resumes the
  // delivery midway, the original queue entry is removed, and the message is
  // lost.
  bool app_connected = _serial && _serial->isConnected();
#ifdef WITH_COMPANION_FRAME_FRAGMENTS
  if (_serial_was_connected && !app_connected) resetFragmentSession();
#else
  if (_serial_was_connected && !app_connected) resetAppSyncState();
#endif
  if (_serial_was_connected && !app_connected) _app_info_len = 0;
  _serial_was_connected = app_connected;

  if (_pending_reboot_at && millisHasNowPassed(_pending_reboot_at) &&
      !deferForAppDrain(_pending_reboot_at)) {
#ifdef NRF52_PLATFORM
    bool ble_busy = _serial && _serial->isConnected();
#else
    bool ble_busy = false;
#endif
    bool has_dirty = dirty_contacts_expiry || dirty_prefs_expiry;
    if (ble_busy && has_dirty && !millisHasNowPassed(_pending_reboot_deadline)) {
      _serial->disconnect();  // NRF52: kick BLE so flash is available for save
      _pending_reboot_at = futureMillis(500);  // wait for disconnect to complete
    } else {
      _pending_reboot_at = 0;
      if (!ble_busy) {
        if (dirty_contacts_expiry) { saveContacts(); dirty_contacts_expiry = 0; }
        if (dirty_prefs_expiry)    { savePrefs();    dirty_prefs_expiry    = 0; }
      }
#ifdef ESP32
      SPIFFS.end();
#endif
      board.reboot();
    }
  }
  if (_pending_poweroff_at && millisHasNowPassed(_pending_poweroff_at) &&
      !deferForAppDrain(_pending_poweroff_at)) {
#ifdef NRF52_PLATFORM
    bool ble_busy = _serial && _serial->isConnected();
#else
    bool ble_busy = false;
#endif
    bool has_dirty = dirty_contacts_expiry || dirty_prefs_expiry;
    if (ble_busy && has_dirty && !millisHasNowPassed(_pending_poweroff_deadline)) {
      _serial->disconnect();  // NRF52: kick BLE so flash is available for save
      _pending_poweroff_at = futureMillis(500);  // wait for disconnect to complete
    } else {
      _pending_poweroff_at = 0;
      if (!ble_busy) {
        if (dirty_contacts_expiry) { saveContacts(); dirty_contacts_expiry = 0; }
        if (dirty_prefs_expiry)    { savePrefs();    dirty_prefs_expiry    = 0; }
      }
#ifdef ESP32
      SPIFFS.end();
#endif
      board.powerOff();
    }
  }


  if (_cli_rescue) {
    checkCLIRescueCmd();
  } else if (_serial) {
    checkSerialInterface();
#if defined(ENABLE_WIFI_INTERFACE) && defined(RP2040_PLATFORM) && !defined(ENABLE_USB_INTERFACE)
    // RP2040 WiFi builds are headless and have no way into the rescue CLI (that needs a
    // display + long-press), so serve config commands on the otherwise unused USB serial
    checkCLIRescueCmd();
#endif
  }

  // On NRF52 with SoftDevice, flash writes block if BLE is active (S140 v6.1.1 known issue).
  // Defer until after disconnect; retry every 1 s while connected.
#ifdef NRF52_PLATFORM
  bool ble_busy = _serial && _serial->isConnected();
#else
  bool ble_busy = false;
#endif
  // Heartbeat: the restore verdict rides along, since the boot line that reports
  // it is easily lost (the USB CDC re-enumerates across a reset).
  if (millisHasNowPassed(offline_beat)) {
    offline_beat = futureMillis(30000);
    OQ_LOG("alive: link=%d store=%d next=%lu acked=%lu queue=%d restored=%u lost=%u\n",
           ble_busy ? 1 : 0, offline_store.usable() ? 1 : 0,
           (unsigned long)offline_store.nextSeq(), (unsigned long)offline_store.acked(),
           offline_queue_len, (unsigned)offline_last_restored, (unsigned)offline_last_lost);
  }
  if (dirty_contacts_expiry && millisHasNowPassed(dirty_contacts_expiry)) {
    if (!ble_busy) {
      saveContacts();
      dirty_contacts_expiry = 0;
    } else {
      dirty_contacts_expiry = futureMillis(1000);
    }
  }
  if (dirty_prefs_expiry && millisHasNowPassed(dirty_prefs_expiry)) {
    if (!ble_busy) {
      savePrefs();
      dirty_prefs_expiry = 0;
    } else {
      dirty_prefs_expiry = futureMillis(1000);
    }
  }

  // The offline store is written in the same window, never with a BLE link up:
  // a flash write stalls the SoftDevice. A wired link on an nRF52 USB build is
  // not a BLE link, and its isConnected() is always true, so it must not block.
#if defined(NRF52_PLATFORM) && defined(BLE_PIN_CODE)
  if (!ble_busy) serviceOfflineStore();
#else
  serviceOfflineStore();
#endif

  if (_listener && _serial) _listener->onAppConnection(_serial->isConnected());
}

bool MyMesh::advert() {
  mesh::Packet* pkt;
  if (_prefs.advert_loc_policy == ADVERT_LOC_NONE) {
    pkt = createSelfAdvert(_prefs.node_name);
  } else {
    pkt = createSelfAdvert(_prefs.node_name, sensors.node_lat, sensors.node_lon);
  }
  if (pkt) {
    sendZeroHop(pkt);
    return true;
  } else {
    return false;
  }
}

#ifdef WITH_COMPANION_CLI

void MyMesh::sendCliReplyPM(const ContactInfo& to, const char* buf) {
  // static: loop_task is single-threaded; these are never called re-entrantly.
  // Without static, chunks plus handleRemoteCLI's buf[512]+cmdBuf[256]
  // overflows the 4096B FreeRTOS loop_task stack → immediate hard fault, no log output.
  static cli_reply::Chunks chunks;
  static char text[160];
  if (cli_reply::split(buf, cli_reply::MAX_CHUNK_TEXT, chunks) !=
      cli_reply::SplitResult::Ok) {
    MESH_DEBUG_PRINTLN("CLI: reply is too long to split safely");
    return;
  }
  uint32_t ack_dummy, timeout_dummy;
  for (size_t i = 0; i < chunks.count; i++) {
    if (chunks.count > 1)
      snprintf(text, sizeof(text), "[%u/%u] %s", (unsigned)(i + 1),
               (unsigned)chunks.count, chunks.text[i]);
    else
      strncpy(text, chunks.text[i], sizeof(text) - 1);
    text[sizeof(text) - 1] = '\0';
    sendMessage(to, getRTCClock()->getCurrentTimeUnique(), 0, text, ack_dummy, timeout_dummy);
  }
}

void MyMesh::sendCliReplyChannel(uint8_t ch_idx, const char* buf, bool mirror_ui) {
  static cli_reply::Chunks chunks;
  static char text[200];
  // Canonical v3 frame; a legacy app gets the v3 -> legacy collapse at delivery.
  const size_t frame_header_size = 11;
  const size_t chunk_capacity = cli_reply::channelChunkCapacity(
      MAX_FRAME_SIZE, frame_header_size, strlen(_prefs.node_name), strlen(buf));
  if (cli_reply::split(buf, chunk_capacity, chunks) != cli_reply::SplitResult::Ok) {
    MESH_DEBUG_PRINTLN("CLI: channel reply is too long to split safely");
    return;
  }
  uint32_t now = getRTCClock()->getCurrentTimeUnique();
  const char* channel_name = "TerminalCLI";
#ifdef DISPLAY_CLASS
  if (mirror_ui) {
    ChannelDetails ch;
    if (getChannel(ch_idx, ch) && ch.name[0]) channel_name = ch.name;
  }
#endif

  for (size_t i = 0; i < chunks.count; i++) {
    int text_length;
    if (chunks.count > 1)
      text_length = snprintf(text, sizeof(text), "%s: [%u/%u] %s", _prefs.node_name,
                             (unsigned)(i + 1), (unsigned)chunks.count,
                             chunks.text[i]);
    else
      text_length = snprintf(text, sizeof(text), "%s: %s", _prefs.node_name,
                             chunks.text[i]);
    if (text_length < 0 || (size_t)text_length >= sizeof(text) ||
        (size_t)text_length > MAX_FRAME_SIZE - frame_header_size) {
      MESH_DEBUG_PRINTLN("CLI: channel reply chunk exceeds frame capacity");
      return;
    }

    int fi = 0;
    out_frame[fi++] = RESP_CODE_CHANNEL_MSG_RECV_V3;
    out_frame[fi++] = 0;  // SNR (synthetic)
    out_frame[fi++] = 0;  // reserved
    out_frame[fi++] = 0;  // reserved
    out_frame[fi++] = ch_idx;
    out_frame[fi++] = 0; // synthetic local reply, 0 LoRa hops
    out_frame[fi++] = TXT_TYPE_PLAIN;
    memcpy(&out_frame[fi], &now, 4); fi += 4;
    if ((size_t)fi != frame_header_size) {
      MESH_DEBUG_PRINTLN("CLI: unexpected channel reply header size");
      return;
    }
    memcpy(&out_frame[fi], text, (size_t)text_length);
    fi += text_length;
    addToOfflineQueue(out_frame, fi);
#ifdef DISPLAY_CLASS
    if (mirror_ui && _listener) _listener->onShowMessage(0, channel_name, text, offline_queue_len, false);
#endif
    now++;  // unique timestamp per chunk
  }

  if (_serial->isConnected()) {
    uint8_t frame[1] = { PUSH_CODE_MSG_WAITING };
    _serial->writeFrame(frame, 1);
    _serial->flushSend();   // the app can only sync once the tickle is out
  }
}

#endif

bool MyMesh::sendGroupMessageWithCyr2LatMap(uint32_t timestamp, mesh::GroupChannel& channel, const char* sender_name,
                                            const char* text, int text_len, const char* original_text,
                                            int original_len, bool record_map) {
  uint8_t temp[5 + MAX_TEXT_LEN + 32];
  memcpy(temp, &timestamp, 4);
  temp[4] = 0;  // TXT_TYPE_PLAIN

  sprintf((char *)&temp[5], "%s: ", sender_name);
  char* ep = strchr((char *)&temp[5], 0);
  int prefix_len = ep - (char *)&temp[5];

  if (text_len + prefix_len > MAX_TEXT_LEN) text_len = MAX_TEXT_LEN - prefix_len;
  memcpy(ep, text, text_len);
  ep[text_len] = 0;

  auto pkt = createGroupDatagram(PAYLOAD_TYPE_GRP_TXT, channel, temp, 5 + prefix_len + text_len);
  if (!pkt) return false;

  if (record_map) {
    Cyr2LatChannelMap& map = _cyr2lat_channel_maps[_next_cyr2lat_channel_map];
    _next_cyr2lat_channel_map = (_next_cyr2lat_channel_map + 1) % CYR2LAT_CHANNEL_MAP_SIZE;

    pkt->calculatePacketHash(map.transformed_hash);

    uint8_t original[5 + MAX_TEXT_LEN + 32];
    memcpy(original, &timestamp, 4);
    original[4] = 0;
    sprintf((char *)&original[5], "%s: ", sender_name);
    char* original_ep = strchr((char *)&original[5], 0);
    int original_prefix_len = original_ep - (char *)&original[5];
    if (original_len + original_prefix_len > MAX_TEXT_LEN) {
      original_len = MAX_TEXT_LEN - original_prefix_len;
      if (original_len < 0) original_len = 0;
    }
    memcpy(original_ep, original_text, original_len);
    original_ep[original_len] = 0;

    int payload_len = 0;
    memcpy(&map.original_payload[payload_len], channel.hash, PATH_HASH_SIZE);
    payload_len += PATH_HASH_SIZE;
    payload_len += mesh::Utils::encryptThenMAC(channel.secret, &map.original_payload[payload_len],
                                               original, 5 + original_prefix_len + original_len);
    map.original_payload_len = payload_len;
  }

  TransportKey default_scope;
  memcpy(&default_scope.key, _prefs.default_scope_key, sizeof(default_scope.key));
  auto scope = send_scope.isNull() ? &default_scope : &send_scope;

  pkt->header &= ~PH_ROUTE_MASK;
  if (scope->isNull()) {
    pkt->header |= ROUTE_TYPE_FLOOD;
  } else {
    pkt->header |= ROUTE_TYPE_TRANSPORT_FLOOD;
    pkt->transport_codes[0] = scope->calcTransportCode(pkt);
    pkt->transport_codes[1] = 0;
  }
  pkt->setPathHashSizeAndCount(_prefs.path_hash_mode + 1, 0);

  getTables()->markSeen(pkt);  // mark own packet as seen so a repeater echo is dropped at dedupe
  sendPacket(pkt, 1);
  return true;
}

int MyMesh::mapCyr2LatChannelRawLog(const uint8_t* raw, int len, uint8_t* mapped, int mapped_size) {
  if (len < 2) return 0;

  mesh::Packet pkt;
  if (!pkt.readFrom(raw, len) || pkt.getPayloadType() != PAYLOAD_TYPE_GRP_TXT) return 0;

  uint8_t hash[MAX_HASH_SIZE];
  pkt.calculatePacketHash(hash);
  for (int i = 0; i < CYR2LAT_CHANNEL_MAP_SIZE; i++) {
    Cyr2LatChannelMap& map = _cyr2lat_channel_maps[i];
    if (map.original_payload_len == 0 || memcmp(hash, map.transformed_hash, MAX_HASH_SIZE) != 0) continue;

    pkt.payload_len = map.original_payload_len;
    memcpy(pkt.payload, map.original_payload, map.original_payload_len);
    if (pkt.getRawLength() > mapped_size) return 0;
    return pkt.writeTo(mapped);
  }
  return 0;
}

// Lower-case the command verb (and subcommand for multi-word prefixes like
// get/set/clock/timesync/clear/wifi/help) while preserving the case of
// any arguments that follow.
static void normalizeCliCommand(char* buf) {
  if (!buf || !*buf) return;
  // skip leading spaces
  char* p = buf;
  while (*p == ' ') p++;
  // lowercase the first word (command verb)
  char* wordStart = p;
  while (*p && *p != ' ') { *p = tolower((unsigned char)*p); p++; }
  if (!*p) return;  // single-word command, done
  // Check if the verb is one that takes a subcommand
  size_t verbLen = p - wordStart;
  static const char* const multiWordVerbs[] = {
    "get", "set", "clock", "timesync", "clear", "wifi", "help"
  };
  bool hasSubCmd = false;
  for (size_t i = 0; i < sizeof(multiWordVerbs) / sizeof(multiWordVerbs[0]); i++) {
    if (verbLen == strlen(multiWordVerbs[i]) && memcmp(wordStart, multiWordVerbs[i], verbLen) == 0) {
      hasSubCmd = true;
      break;
    }
  }
  if (!hasSubCmd) return;  // no subcommand to lowercase
  // skip spaces between verb and subcommand
  while (*p == ' ') p++;
  // lowercase the subcommand (second word)
  while (*p && *p != ' ') { *p = tolower((unsigned char)*p); p++; }
  // everything after the subcommand keeps its original case
}

#ifdef WITH_COMPANION_CLI

// Every way into the companion CLI (PIN message, the app's CLI screen,
// TerminalCLI) runs the same chain: our companion commands, then upstream's
// companion CLI (radio prefs, board, name, pin, wifi, tz.offset), then the CLI
// shared with the other roles. Upstream's commands come in with each merge.
void MyMesh::runCliChain(uint32_t sender_ts, const char* cmd, char* buf, bool is_remote) {
  if (handleCliCmd(sender_ts, cmd, buf, is_remote)) return;
  if (handleCommand(cmd, sender_ts, buf)) return;
  if (_cli) _cli->handleCommand(sender_ts, const_cast<char*>(cmd), buf);
  else      strcpy(buf, "ERR: CLI not initialized");
}

bool MyMesh::handleCliCmd(uint32_t sender_ts, const char* cmd, char* buf, bool is_remote) {
  if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
    snprintf(buf, 512,
      "Commands: help <topic>\n"
      " radio - LoRa parameters\n"
      " sys   - System, time, PIN, reboot\n"
      " app   - Features & messaging\n"
      " all   - Full command list\n"
      "Quick: pin, ver, reboot, get radio");
  } else if (strcmp(cmd, "help radio") == 0) {
    snprintf(buf, 512,
      "Radio parameters:\n"
      "get/set radio [freq, bw, sf, cr, txpower]\n"
      "get/set txdelay, rxdelay, airtime");
  } else if (strcmp(cmd, "help sys") == 0) {
    snprintf(buf, 512,
      "System & Timesync:\n"
      "pin - show CLI/BLE PIN; get/set ble.pin <0|PIN>\n"
      "get/set ble.prefix [on|off]\n"
      "timesync [on|off|adverts on/off|msgs on/off|reset|params]\n"
      "set timesync.<cluster|drift|jump|back|age|quiet> <s>\n"
      "clock / time - RTC status\n"
      "reboot / poweroff - restart / shut down\n"
      "neighbors - heard node list\n"
      "stats-core / stats-packets / stats-radio\n"
      "log / tempradio / powersaving / clear stats\n"
      "ver / board - firmware & hardware info");
  } else if (strcmp(cmd, "help app") == 0) {
    snprintf(buf, 512,
      "App & Features:\n"
      "get/set remote.cli [pin|flag|both|off], remote.cli.fav [on|off]\n"
      "get/set cli.pin [<8 hex>|new], terminal.cli [on|off]\n"
      "get/set cyr2lat.channels|cyr2lat.contacts [on|off]\n"
#ifdef WITH_MCOTXT
      "get/set mcotxt [on|off]\n"
#endif
#ifdef WITH_MCMP_DETECT
      "get/set mcmp.detect [on|off]\n"
#endif
#ifdef WITH_AEIC_DETECT
      "get/set aeic.detect [on|off]\n"
#endif
#ifdef WITH_MCOIMG_DETECT
      "get/set mcoimg.detect [on|off]\n"
#endif
#ifdef WITH_COMMS_SWITCHING
      "get/set comms [ble|usb|wifi]\n"
#endif
#if defined(WITH_COMMS_SWITCHING) && defined(ENABLE_WIFI_INTERFACE)
      "wifi list|scan|add|del|ip|port|connect|status\n"
#endif
    );
  } else if (strcmp(cmd, "help all") == 0) {
    snprintf(buf, 512,
      "Full command list:\n"
      "Radio: get/set radio, txdelay, rxdelay, airtime\n"
      "Sys: pin, get/set ble.pin|ble.prefix, timesync [on|off|params], clock, reboot, poweroff, neighbors, stats, ver, board\n"
      "App: get/set remote.cli|terminal.cli|cyr2lat.channels|contacts"
#ifdef WITH_MCOTXT
      ", mcotxt"
#endif
#ifdef WITH_MCMP_DETECT
      ", mcmp.detect"
#endif
#ifdef WITH_AEIC_DETECT
      ", aeic.detect"
#endif
#ifdef WITH_MCOIMG_DETECT
      ", mcoimg.detect"
#endif
#ifdef WITH_COMMS_SWITCHING
      ", comms"
#endif
#if defined(WITH_COMMS_SWITCHING) && defined(ENABLE_WIFI_INTERFACE)
      ", wifi"
#endif
    );
  } else if (strncmp(cmd, "help ", 5) == 0) {
    snprintf(buf, 512, "Unknown topic '%s'. Use: help [radio|sys|app|all]", cmd + 5);
  } else if (strcmp(cmd, "pin") == 0) {
    if (_prefs.ble_pin == 0)
      snprintf(buf, 512, "CLI PIN: %s  BLE PIN: %06lu (auto, new random on reboot)", _prefs.cli_pin, (unsigned long)_active_ble_pin);
    else
      snprintf(buf, 512, "CLI PIN: %s  BLE PIN: %06lu", _prefs.cli_pin, (unsigned long)_prefs.ble_pin);

  } else if (strcmp(cmd, "get cli.pin") == 0) {
    snprintf(buf, 512, "cli.pin: %s", _prefs.cli_pin);
  } else if (strcmp(cmd, "set cli.pin new") == 0) {
    ensureCliPin(true);
    snprintf(buf, 512, "cli.pin: %s (new)", _prefs.cli_pin);
  } else if (strncmp(cmd, "set cli.pin ", 12) == 0) {
    if (setCliPin(cmd + 12)) snprintf(buf, 512, "cli.pin: %s", _prefs.cli_pin);
    else strcpy(buf, "ERR: cli.pin is 8 hex digits, or 'new'");

  } else if (strcmp(cmd, "get ble.pin") == 0) {
    if (_prefs.ble_pin == 0)
      snprintf(buf, 512, "ble.pin: auto (current session: %06lu)", (unsigned long)_active_ble_pin);
    else
      snprintf(buf, 512, "ble.pin: %06lu", (unsigned long)_prefs.ble_pin);
  } else if (strncmp(cmd, "set ble.pin ", 12) == 0) {
    uint32_t pin = (uint32_t)atol(cmd + 12);
    if (pin == 0 || (pin >= 100000 && pin <= 999999)) {
      _prefs.ble_pin = pin;
      if (pin != 0) _active_ble_pin = pin;  // don't zero out display — pin unchanged until reboot
      dirty_prefs_expiry = futureMillis(LAZY_PREFS_WRITE_DELAY);
      if (pin == 0)
        strcpy(buf, "ble.pin: auto (new random on reboot)");
      else
        snprintf(buf, 512, "ble.pin: %06lu (reboot to apply)", (unsigned long)pin);
    } else {
      strcpy(buf, "ERR: pin must be 0 (auto/random) or 100000-999999");
    }

  } else if (strcmp(cmd, "get ble.prefix") == 0) {
    // What the next boot will advertise; a prefs file without the key keeps the prefix.
    snprintf(buf, 512, "ble.prefix: %s", blePrefixEnabled(&_prefs) ? "on" : "off");
  } else if (strncmp(cmd, "set ble.prefix ", 15) == 0) {
    const char* val = cmd + 15;
    if (strcmp(val, "on") == 0 || strcmp(val, "off") == 0) {
      bool on = strcmp(val, "on") == 0;
      _prefs.ui_ble_prefix = on ? BLE_PREFIX_ON : BLE_PREFIX_OFF;
      dirty_prefs_expiry = futureMillis(LAZY_PREFS_WRITE_DELAY);
      // The name goes to the BLE stack once at begin(): nothing renames a live advertisement.
      snprintf(buf, 512, "ble.prefix: %s (reboot to apply)", on ? "on" : "off");
    } else {
      strcpy(buf, "ERR: use on or off");
    }


  } else if (strcmp(cmd, "timesync") == 0) {
    _ts.buildReply(buf, 512, getRTCClock());
    char flags[64];
    snprintf(flags, sizeof(flags), "\nAdverts: %s  Msgs: %s",
             _ts_from_adverts ? "on" : "off", _ts_from_messages ? "on" : "off");
    strncat(buf, flags, 511 - strlen(buf));

  } else if (strcmp(cmd, "timesync on") == 0) {
    _ts_from_adverts = _ts_from_messages = true;
    strcpy(buf, "timesync: on (adverts+msgs)");
  } else if (strcmp(cmd, "timesync off") == 0) {
    _ts_from_adverts = _ts_from_messages = false;
    strcpy(buf, "timesync: off");
  } else if (strcmp(cmd, "timesync adverts on") == 0) {
    _ts_from_adverts = true;
    strcpy(buf, "timesync adverts: on");
  } else if (strcmp(cmd, "timesync adverts off") == 0) {
    _ts_from_adverts = false;
    strcpy(buf, "timesync adverts: off");
  } else if (strcmp(cmd, "timesync msgs on") == 0) {
    _ts_from_messages = true;
    strcpy(buf, "timesync msgs: on");
  } else if (strcmp(cmd, "timesync msgs off") == 0) {
    _ts_from_messages = false;
    strcpy(buf, "timesync msgs: off");
  } else if (strcmp(cmd, "timesync reset") == 0) {
    _ts.reset();
    strcpy(buf, "timesync: state reset (params kept)");

  } else if (strcmp(cmd, "timesync params") == 0) {
    snprintf(buf, 512, "cluster: %lus (default %us)\ndrift:   %lus (default %us)\njump:    %lus (default %us)\nback:    %lus (default %us)\nage:     %lus (default %us)\nquiet:   %d peers (default %d)",
             (unsigned long)_ts.cluster_window,  TS_CLUSTER_WINDOW,
             (unsigned long)_ts.drift_threshold, TS_DRIFT_THRESHOLD,
             (unsigned long)_ts.max_jump,         TS_MAX_JUMP,
             (unsigned long)_ts.max_back,         TS_MAX_BACK,
             (unsigned long)_ts.max_age,          TS_MAX_AGE,
             _ts.quiet_quorum, TS_QUIET_QUORUM);
  } else if (strncmp(cmd, "set timesync.cluster ", 21) == 0) {
    _ts.cluster_window = atoi(cmd + 21);
    snprintf(buf, 512, "cluster_window: %lus", (unsigned long)_ts.cluster_window);
  } else if (strncmp(cmd, "set timesync.drift ", 19) == 0) {
    _ts.drift_threshold = atoi(cmd + 19);
    snprintf(buf, 512, "drift_threshold: %lus", (unsigned long)_ts.drift_threshold);
  } else if (strncmp(cmd, "set timesync.jump ", 18) == 0) {
    _ts.max_jump = atoi(cmd + 18);
    snprintf(buf, 512, "max_jump: %lus", (unsigned long)_ts.max_jump);
  } else if (strncmp(cmd, "set timesync.back ", 18) == 0) {
    _ts.max_back = atoi(cmd + 18);
    snprintf(buf, 512, "max_back: %lus", (unsigned long)_ts.max_back);
  } else if (strncmp(cmd, "set timesync.age ", 17) == 0) {
    _ts.max_age = atoi(cmd + 17);
    snprintf(buf, 512, "max_age: %lus", (unsigned long)_ts.max_age);
  } else if (strncmp(cmd, "set timesync.quiet ", 19) == 0) {
    _ts.quiet_quorum = atoi(cmd + 19);
    snprintf(buf, 512, "quiet_quorum: %d", _ts.quiet_quorum);

  } else if (strcmp(cmd, "get remote.cli") == 0) {
    static const char* names[] = { "pin", "flag", "both", "off" };
    snprintf(buf, 512, "remote.cli: %s", names[_prefs.rcli_mode <= REMOTE_CLI_OFF ? _prefs.rcli_mode : 0]);
  } else if (strncmp(cmd, "set remote.cli ", 15) == 0) {
    const char* v = cmd + 15;
    int m = -1;
    if (strcmp(v, "pin") == 0 || strcmp(v, "on") == 0) m = REMOTE_CLI_PIN;
    else if (strcmp(v, "flag") == 0) m = REMOTE_CLI_FLAG;
    else if (strcmp(v, "both") == 0) m = REMOTE_CLI_BOTH;
    else if (strcmp(v, "off") == 0)  m = REMOTE_CLI_OFF;
    if (m < 0) {
      strcpy(buf, "ERR: remote.cli pin|flag|both|off");
    } else {
      _prefs.rcli_mode = m;
      savePrefs();
      if (m == REMOTE_CLI_OFF && is_remote) strcpy(buf, "remote.cli: off (this was the last remote command)");
      else snprintf(buf, 512, "remote.cli: %s", v);
    }
  } else if (strcmp(cmd, "get remote.cli.fav") == 0) {
    snprintf(buf, 512, "remote.cli.fav: %s", _prefs.rcli_fav ? "off" : "on");
  } else if (strcmp(cmd, "set remote.cli.fav on") == 0 || strcmp(cmd, "set remote.cli.fav off") == 0) {
    _prefs.rcli_fav = strcmp(cmd + 19, "off") == 0 ? 1 : 0;
    savePrefs();
    snprintf(buf, 512, "remote.cli.fav: %s", _prefs.rcli_fav ? "off" : "on");

  } else if (strcmp(cmd, "get terminal.cli") == 0) {
    snprintf(buf, 512, "terminal.cli: %s", _terminal_cli_enabled ? "on" : "off");
  } else if (strcmp(cmd, "set terminal.cli on") == 0) {
    if (!_terminal_cli_enabled) {
      if (addChannel("TerminalCLI", TERMINAL_CLI_PSK)) {
        saveChannels();
        _terminal_cli_enabled = true;
        strcpy(buf, "terminal.cli: on (channel registered)");
      } else {
        strcpy(buf, "ERR: addChannel failed (slots full?)");
      }
    } else {
      strcpy(buf, "terminal.cli: already on");
    }
  } else if (strcmp(cmd, "set terminal.cli off") == 0) {
    bool found = false;
    for (int i = 0; i < MAX_GROUP_CHANNELS; i++) {
      ChannelDetails ch;
      if (getChannel(i, ch) && ch.name[0] && strcmp(ch.name, "TerminalCLI") == 0) {
        ChannelDetails empty{};
        setChannel(i, empty);
        found = true;
        break;
      }
    }
    if (found) saveChannels();
    _terminal_cli_enabled = false;
    strcpy(buf, found ? "terminal.cli: off (channel removed)" : "terminal.cli: off");
  } else if (strcmp(cmd, "get cyr2lat.channels") == 0) {
    snprintf(buf, 512, "cyr2lat.channels: %s", _cyr2lat_channels_enabled ? "on" : "off");
  } else if (strcmp(cmd, "set cyr2lat.channels on") == 0) {
    _cyr2lat_channels_enabled = true;
    _prefs.cyr2lat_channels = 1;
    dirty_prefs_expiry = futureMillis(LAZY_PREFS_WRITE_DELAY);
    strcpy(buf, "cyr2lat.channels: on");
  } else if (strcmp(cmd, "set cyr2lat.channels off") == 0) {
    _cyr2lat_channels_enabled = false;
    _prefs.cyr2lat_channels = 0;
    dirty_prefs_expiry = futureMillis(LAZY_PREFS_WRITE_DELAY);
    strcpy(buf, "cyr2lat.channels: off");
  } else if (strcmp(cmd, "get cyr2lat.contacts") == 0) {
    snprintf(buf, 512, "cyr2lat.contacts: %s", _cyr2lat_contacts_enabled ? "on" : "off");
  } else if (strcmp(cmd, "set cyr2lat.contacts on") == 0) {
    _cyr2lat_contacts_enabled = true;
    _prefs.cyr2lat_contacts = 1;
    dirty_prefs_expiry = futureMillis(LAZY_PREFS_WRITE_DELAY);
    strcpy(buf, "cyr2lat.contacts: on");
  } else if (strcmp(cmd, "set cyr2lat.contacts off") == 0) {
    _cyr2lat_contacts_enabled = false;
    _prefs.cyr2lat_contacts = 0;
    dirty_prefs_expiry = futureMillis(LAZY_PREFS_WRITE_DELAY);
    strcpy(buf, "cyr2lat.contacts: off");

#ifdef WITH_MCOTXT
  } else if (strcmp(cmd, "get mcotxt") == 0) {
    snprintf(buf, 512, "mcotxt: %s", isMCOtxtEnabled() ? "on" : "off");
  } else if (strcmp(cmd, "set mcotxt on") == 0) {
    setMCOtxtEnabled(true);
    strcpy(buf, "mcotxt: on");
  } else if (strcmp(cmd, "set mcotxt off") == 0) {
    setMCOtxtEnabled(false);
    strcpy(buf, "mcotxt: off");
#endif
#ifdef WITH_MCMP_DETECT
  } else if (strcmp(cmd, "get mcmp.detect") == 0) {
    snprintf(buf, 512, "mcmp.detect: %s", isMCMPDetectEnabled() ? "on" : "off");
  } else if (strcmp(cmd, "set mcmp.detect on") == 0) {
    setMCMPDetectEnabled(true);
    strcpy(buf, "mcmp.detect: on");
  } else if (strcmp(cmd, "set mcmp.detect off") == 0) {
    setMCMPDetectEnabled(false);
    strcpy(buf, "mcmp.detect: off");
#endif
#ifdef WITH_AEIC_DETECT
  } else if (strcmp(cmd, "get aeic.detect") == 0) {
    snprintf(buf, 512, "aeic.detect: %s", isAEICDetectEnabled() ? "on" : "off");
  } else if (strcmp(cmd, "set aeic.detect on") == 0) {
    setAEICDetectEnabled(true);
    strcpy(buf, "aeic.detect: on");
  } else if (strcmp(cmd, "set aeic.detect off") == 0) {
    setAEICDetectEnabled(false);
    strcpy(buf, "aeic.detect: off");
#endif
#ifdef WITH_MCOIMG_DETECT
  } else if (strcmp(cmd, "get mcoimg.detect") == 0) {
    snprintf(buf, 512, "mcoimg.detect: %s", isMCOimgDetectEnabled() ? "on" : "off");
  } else if (strcmp(cmd, "set mcoimg.detect on") == 0) {
    setMCOimgDetectEnabled(true);
    strcpy(buf, "mcoimg.detect: on");
  } else if (strcmp(cmd, "set mcoimg.detect off") == 0) {
    setMCOimgDetectEnabled(false);
    strcpy(buf, "mcoimg.detect: off");
#endif

#ifdef WITH_COMPANION_FRAME_FRAGMENTS
  } else if (strcmp(cmd, "get framed.packets") == 0) {
    snprintf(buf, 512, "framed.packets: %s", isFramedPacketsEnabled() ? "on" : "off");
  } else if (strcmp(cmd, "set framed.packets on") == 0) {
    setFramedPacketsEnabled(true);
    strcpy(buf, "framed.packets: on");
  } else if (strcmp(cmd, "set framed.packets off") == 0) {
    setFramedPacketsEnabled(false);
    strcpy(buf, "framed.packets: off");
#endif

#if defined(WITH_COMMS_SWITCHING) && defined(ENABLE_WIFI_INTERFACE)
  } else if (strcmp(cmd, "wifi list") == 0) {
    if (_wifi_prefs.network_count == 0) {
      strcpy(buf, "No WiFi networks stored. Use: wifi add <ssid> <pass>");
    } else {
      int n = 0;
      for (int i = 0; i < _wifi_prefs.network_count; i++)
        n += snprintf(buf + n, 512 - n, "[%d] %s\n", i, _wifi_prefs.networks[i].ssid);
      snprintf(buf + n, 512 - n, "mode: %s  port: %u",
               _wifi_prefs.ip_mode == IP_MODE_STATIC ? "static" : "dhcp",
               (unsigned)(_wifi_prefs.tcp_port ? _wifi_prefs.tcp_port : WIFI_TCP_PORT_DEFAULT));
    }

  } else if (strcmp(cmd, "wifi scan") == 0) {
    strcpy(buf, "Scanning...");
    int n_found = WiFi.scanNetworks();
    if (n_found <= 0) {
      strcpy(buf, "No networks found.");
    } else {
      int n = 0;
      for (int i = 0; i < n_found && n < 500; i++)
        n += snprintf(buf + n, 512 - n, "%s (%d dBm)\n", WiFi.SSID(i).c_str(), WiFi.RSSI(i));
    }
    wifiScanDone();

  } else if (strncmp(cmd, "wifi add ", 9) == 0) {
    char ssid[33] = {}, pass[65] = {};
    const char* p = cmd + 9;
    // Parse SSID: quoted ("My Network") or plain (NoSpaces)
    if (*p == '"') {
      p++;
      int i = 0;
      while (*p && *p != '"' && i < 32) ssid[i++] = *p++;
      if (*p == '"') p++;
    } else {
      int i = 0;
      while (*p && *p != ' ' && i < 32) ssid[i++] = *p++;
    }
    while (*p == ' ') p++;
    // Parse password: quoted or plain (rest of string)
    if (*p == '"') {
      p++;
      int i = 0;
      while (*p && *p != '"' && i < 64) pass[i++] = *p++;
    } else {
      strncpy(pass, p, 64);
    }
    if (ssid[0]) {
      if (_wifi_prefs.network_count >= WIFI_MAX_NETWORKS) {
        strcpy(buf, "ERR: max 5 networks stored");
      } else {
        addWifiNetwork(ssid, pass);
        snprintf(buf, 512, "wifi: saved [%s]", ssid);
      }
    } else {
      strcpy(buf, "Usage: wifi add <ssid> [password]\n  Quotes for spaces: wifi add \"My Network\" \"my pass\"");
    }

  } else if (strncmp(cmd, "wifi del ", 9) == 0) {
    const char* p = cmd + 9;
    char ssid[33] = {};
    if (*p == '"') {
      p++;
      int i = 0;
      while (*p && *p != '"' && i < 32) ssid[i++] = *p++;
    } else {
      strncpy(ssid, p, 32);
    }
    removeWifiNetwork(ssid);
    snprintf(buf, 512, "wifi: removed [%s] (if existed)", ssid);

  } else if (strcmp(cmd, "wifi ip dhcp") == 0) {
    _wifi_prefs.ip_mode = IP_MODE_DHCP;
    saveWifiPrefs();
    strcpy(buf, "wifi ip: dhcp");

  } else if (strncmp(cmd, "wifi ip static ", 15) == 0) {
    char ip[16] = {}, gw[16] = {}, mask[16] = {};
    if (sscanf(cmd + 15, "%15s %15s %15s", ip, gw, mask) == 3) {
      _wifi_prefs.ip_mode = IP_MODE_STATIC;
      strncpy(_wifi_prefs.static_ip,   ip,   sizeof(_wifi_prefs.static_ip) - 1);
      strncpy(_wifi_prefs.static_gw,   gw,   sizeof(_wifi_prefs.static_gw) - 1);
      strncpy(_wifi_prefs.static_mask, mask, sizeof(_wifi_prefs.static_mask) - 1);
      saveWifiPrefs();
      snprintf(buf, 512, "wifi ip: static %s gw %s mask %s", ip, gw, mask);
    } else {
      strcpy(buf, "Usage: wifi ip static <ip> <gateway> <mask>");
    }

  } else if (strncmp(cmd, "wifi port ", 10) == 0) {
    int port = atoi(cmd + 10);
    if (port > 0 && port < 65536) {
      _wifi_prefs.tcp_port = (uint16_t)port;
      saveWifiPrefs();
      snprintf(buf, 512, "wifi port: %d (reboot to apply)", port);
    } else {
      strcpy(buf, "Usage: wifi port <1-65535>");
    }

  } else if (strncmp(cmd, "wifi connect ", 13) == 0) {
    int idx = atoi(cmd + 13);
    if (idx >= 0 && idx < _wifi_prefs.network_count) {
      snprintf(buf, 512, "connecting to [%s]...", _wifi_prefs.networks[idx].ssid);
      switchCommsMode(COMMS_MODE_WIFI, idx);
    } else {
      snprintf(buf, 512, "ERR: index %d out of range (0..%d)", idx, _wifi_prefs.network_count - 1);
    }

  } else if (strcmp(cmd, "wifi status") == 0) {
    const char* mode_str = (_wifi_prefs.comms_mode == COMMS_MODE_WIFI) ? "BLE+WiFi"
                         : (_wifi_prefs.comms_mode == COMMS_MODE_USB)  ? "USB" : "BLE";
    if (isWifiConnecting()) {
      snprintf(buf, 512, "comms: %s (connecting...)", mode_str);
    } else if (_wifi_prefs.comms_mode == COMMS_MODE_WIFI && WiFi.status() == WL_CONNECTED) {
      snprintf(buf, 512, "comms: WiFi  ip: %s  port: %u",
               WiFi.localIP().toString().c_str(),
               (unsigned)(_wifi_prefs.tcp_port ? _wifi_prefs.tcp_port : WIFI_TCP_PORT_DEFAULT));
    } else {
      snprintf(buf, 512, "comms: %s", mode_str);
    }
#endif  // WITH_COMMS_SWITCHING && ENABLE_WIFI_INTERFACE

  } else {
    return false; // not a companion command — delegate to CommonCLI
  }
  return true;
}


// The PIN is a hash of the private key and a random salt, made once and kept in
// prefs: it says nothing about the key and can be replaced (CLI, menu).
void MyMesh::ensureCliPin(bool regenerate) {
  bool valid = strlen(_prefs.cli_pin) == 8;
  for (int i = 0; valid && i < 8; i++) valid = mesh::Utils::isHexChar(_prefs.cli_pin[i]);
  if (valid && !regenerate) return;
  uint8_t prv[PRV_KEY_SIZE];
  self_id.writeTo(prv, PRV_KEY_SIZE);
  uint8_t salt[16];
  getRNG()->random(salt, sizeof(salt));
  uint8_t h[4];
  mesh::Utils::sha256(h, sizeof(h), prv, PRV_KEY_SIZE, salt, sizeof(salt));
  memset(prv, 0, sizeof(prv));
  mesh::Utils::toHex(_prefs.cli_pin, h, sizeof(h));
  for (char* c = _prefs.cli_pin; *c; c++) *c = tolower(*c);
  savePrefs();
}

bool MyMesh::setCliPin(const char* pin) {
  if (strlen(pin) != 8) return false;
  for (int i = 0; i < 8; i++) if (!mesh::Utils::isHexChar(pin[i])) return false;
  for (int i = 0; i < 8; i++) _prefs.cli_pin[i] = tolower(pin[i]);
  _prefs.cli_pin[8] = 0;
  savePrefs();
  return true;
}

MyMesh::RemoteCliPeer* MyMesh::rcliPeer(const ContactInfo& from) {
  RemoteCliPeer* free_p = NULL;
  RemoteCliPeer* oldest = &_rcli_peers[0];
  static const uint8_t zero[4] = {0, 0, 0, 0};
  for (auto& p : _rcli_peers) {
    if (memcmp(p.key, from.id.pub_key, 4) == 0) { p.seen = millis(); return &p; }
    if (!free_p && memcmp(p.key, zero, 4) == 0) free_p = &p;
    if ((int32_t)(p.seen - oldest->seen) < 0) oldest = &p;
  }
  RemoteCliPeer* p = free_p ? free_p : oldest;
  memset(p, 0, sizeof(*p));
  memcpy(p->key, from.id.pub_key, 4);
  p->seen = millis();
  return p;
}

// A command must be newer than the last one from this sender, and - when we
// trust our own clock - close to now: a recorded packet replayed later, after
// the mesh's duplicate filter has forgotten it, must not run again.
bool MyMesh::rcliFresh(RemoteCliPeer* p, uint32_t sender_ts) {
  if (sender_ts <= p->last_ts) return false;
  if (getRTCClock()->isTimeReliable()) {
    uint32_t now = getRTCClock()->getCurrentTime();
    uint32_t d = sender_ts > now ? sender_ts - now : now - sender_ts;
    if (d > RCLI_MAX_SKEW) return false;
  }
  p->last_ts = sender_ts;
  return true;
}

// 1: "<PIN> <command>" with the right PIN, *cmd set; 0: no PIN or a wrong one
// (an ordinary message); -1: drop - locked out, replayed or stale.
int MyMesh::rcliCheckPin(const ContactInfo& from, const char* text, uint32_t sender_ts, const char** cmd) {
  if (strlen(text) < 10 || text[8] != ' ') return 0;
  for (int i = 0; i < 8; i++) if (!mesh::Utils::isHexChar(text[i])) return 0;
  RemoteCliPeer* p = rcliPeer(from);
  if (p->lock_until && (int32_t)(millis() - p->lock_until) < 0) return -1;
  p->lock_until = 0;
  if (!_prefs.cli_pin[0] || strncasecmp(text, _prefs.cli_pin, 8) != 0) {
    if (++p->bad >= RCLI_MAX_BAD) {
      p->bad = 0;
      p->lock_until = millis() + RCLI_LOCK_MS;
      if (!p->lock_until) p->lock_until = 1;
    }
    return 0;
  }
  p->bad = 0;
  if (!rcliFresh(p, sender_ts)) return -1;
  *cmd = text + 9;
  return 1;
}

// Whoever knows the PIN stays in the contacts: auto-add overwrites only
// contacts that are not favourites.
void MyMesh::rcliAccepted(const ContactInfo& from) {
  if (_prefs.rcli_fav) return;
  ContactInfo* c = lookupContactByPubKey(from.id.pub_key, PUB_KEY_SIZE);
  if (c && !c->isFav()) {
    c->flags |= 0x01;
    dirty_contacts_expiry = futureMillis(LAZY_CONTACTS_WRITE_DELAY);
  }
}

void MyMesh::handleRemoteCLI(const ContactInfo& from, uint32_t sender_ts, const char* cmd) {
  char from_hex[9];
  mesh::Utils::toHex(from_hex, from.id.pub_key, 4);

  // static: saves 768B (cmdBuf+buf) from the loop_task stack on every call
  static char cmdBuf[256];
  strncpy(cmdBuf, cmd, sizeof(cmdBuf) - 1);
  cmdBuf[sizeof(cmdBuf) - 1] = '\0';
  char* e = cmdBuf + strlen(cmdBuf);
  while (e > cmdBuf && (e[-1] == ' ' || e[-1] == '\r' || e[-1] == '\n')) *--e = '\0';
  cmd = cmdBuf;
  normalizeCliCommand(cmdBuf);

  MESH_DEBUG_PRINTLN("CLI/PM from=%s cmd='%s'", from_hex, cmd);

  static char buf[512];
  buf[0] = '\0';
  if (strcmp(cmd, "reboot") == 0) {
    sendCliReplyPM(from, "rebooting in 1s...");
    _pending_reboot_at = futureMillis(1000);
    _pending_reboot_deadline = futureMillis(30000);
    _app_drain_until = 0;   // fresh command: open a new drain window
    return;
  } else if (strcmp(cmd, "poweroff") == 0 || strcmp(cmd, "shutdown") == 0) {
    sendCliReplyPM(from, "powering off...");
    _pending_poweroff_at = futureMillis(500);
    _pending_poweroff_deadline = futureMillis(30000);
    _app_drain_until = 0;   // fresh command: open a new drain window
    return;
  } else {
    runCliChain(sender_ts, cmd, buf, true);
  }
  MESH_DEBUG_PRINTLN("CLI/PM reply(%zu): '%s'", strlen(buf), buf);
  if (buf[0]) sendCliReplyPM(from, buf);
}

void MyMesh::handleTerminalCLI(uint8_t ch_idx, uint32_t sender_ts, const char* cmd,
                               bool mirror_ui_reply) {
  static char cmdBuf[256];
  strncpy(cmdBuf, cmd, sizeof(cmdBuf) - 1);
  cmdBuf[sizeof(cmdBuf) - 1] = '\0';
  char* e = cmdBuf + strlen(cmdBuf);
  while (e > cmdBuf && (e[-1] == ' ' || e[-1] == '\r' || e[-1] == '\n')) *--e = '\0';
  cmd = cmdBuf;
  normalizeCliCommand(cmdBuf);

  MESH_DEBUG_PRINTLN("CLI/Terminal ch=%u cmd='%s'", ch_idx, cmd);

  static char buf[512];
  buf[0] = '\0';
  if (strcmp(cmd, "reboot") == 0) {
    strcpy(buf, "rebooting in 1s...");
    _pending_reboot_at = futureMillis(1000);
    _pending_reboot_deadline = futureMillis(30000);
    _app_drain_until = 0;   // fresh command: open a new drain window
  } else if (strcmp(cmd, "poweroff") == 0 || strcmp(cmd, "shutdown") == 0) {
    strcpy(buf, "powering off...");
    _pending_poweroff_at = futureMillis(500);
    _pending_poweroff_deadline = futureMillis(30000);
    _app_drain_until = 0;   // fresh command: open a new drain window
  } else {
    runCliChain(sender_ts, cmd, buf, false);
  }
  MESH_DEBUG_PRINTLN("CLI/Terminal reply(%zu): '%s'", strlen(buf), buf);
  if (buf[0]) sendCliReplyChannel(ch_idx, buf, mirror_ui_reply);
}

#endif

void MyMesh::setCyr2LatChannelsEnabled(bool enabled) {
  _cyr2lat_channels_enabled = enabled;
  _prefs.cyr2lat_channels = enabled ? 1 : 0;
  dirty_prefs_expiry = futureMillis(LAZY_PREFS_WRITE_DELAY);
}

void MyMesh::setCyr2LatContactsEnabled(bool enabled) {
  _cyr2lat_contacts_enabled = enabled;
  _prefs.cyr2lat_contacts = enabled ? 1 : 0;
  dirty_prefs_expiry = futureMillis(LAZY_PREFS_WRITE_DELAY);
}

static size_t compatSenderPrefixLength(const char* text, bool has_name) {
  if (!has_name) return 0;
  const char* separator = strstr(text, ": ");
  return separator == nullptr || separator == text ? 0 : (size_t)(separator + 2 - text);
}

static mco_compat::Result transformCompatPayload(
    const char* text, bool has_name, char* output, size_t output_size,
    const mco_compat::Options& options) {
  mco_compat::Result result = { false, false, 0 };
  if (!text || !output || output_size == 0) return result;
  const size_t prefix_length = compatSenderPrefixLength(text, has_name);
  if (prefix_length >= output_size) {
    memcpy(output, text, output_size - 1);
    output[output_size - 1] = '\0';
    result.truncated = true;
    result.length = output_size - 1;
    return result;
  }
  if (prefix_length != 0) memcpy(output, text, prefix_length);
  const mco_compat::Result body = mco_compat::transform(
      text + prefix_length, output + prefix_length,
      output_size - prefix_length, options);
  result = body;
  result.length += prefix_length;
  return result;
}

bool MyMesh::transformCompatText(const char* text, bool has_name, char* output,
                                 size_t output_size) {
  mco_compat::Options options = { false, false, false };
#ifdef WITH_MCOTXT
  options.mcotxt = isMCOtxtEnabled();
#endif
#ifdef WITH_MCMP_DETECT
  options.mcmp = isMCMPDetectEnabled();
#endif
#ifdef WITH_MCOIMG_DETECT
  options.mcoimg = isMCOimgDetectEnabled();
#endif
  return transformCompatPayload(text, has_name, output, output_size, options).changed;
}

struct CompatTextFrameLayout {
  size_t header_length;
  bool has_sender_name;
};

static bool compatTextFrameLayout(const uint8_t* frame, size_t length,
                                  CompatTextFrameLayout& layout) {
  if (frame == nullptr || length == 0) return false;
  size_t text_type_position = 0;
  switch (frame[0]) {
    case RESP_CODE_CONTACT_MSG_RECV:
      if (length < 13) return false;
      text_type_position = 8;
      layout = { 13, false };
      break;
    case RESP_CODE_CONTACT_MSG_RECV_V3:
      if (length < 16) return false;
      text_type_position = 11;
      layout = { 16, false };
      break;
    case RESP_CODE_CHANNEL_MSG_RECV:
      if (length < 8) return false;
      text_type_position = 3;
      layout = { 8, true };
      break;
    case RESP_CODE_CHANNEL_MSG_RECV_V3:
      if (length < 11) return false;
      text_type_position = 6;
      layout = { 11, true };
      break;
    default:
      return false;
  }

  const uint8_t text_type = frame[text_type_position];
  if (text_type != TXT_TYPE_PLAIN && text_type != TXT_TYPE_SIGNED_PLAIN) return false;
  if (!layout.has_sender_name && text_type == TXT_TYPE_SIGNED_PLAIN)
    layout.header_length += 4;  // sender key prefix appended by queueMessage()
  return layout.header_length <= length;
}

static int formatCompatPart(const uint8_t* header, size_t header_length,
                            const char* prefix, size_t prefix_length,
                            const char* text, size_t text_length,
                            size_t part_index, size_t& part_count,
                            uint8_t output[]) {
  part_count = 0;
  if (header == nullptr || text == nullptr || output == nullptr ||
      header_length >= MAX_FRAME_SIZE) return 0;
  size_t max_bytes = MAX_FRAME_SIZE - header_length;
  if (max_bytes > MAX_TEXT_LEN) max_bytes = MAX_TEXT_LEN;

#ifdef WITH_MCOTXT
  mcotxt::TextPart parts[mcotxt::kMaxParts];
  part_count = mcotxt::splitForApp(text, text_length, max_bytes,
                                   prefix_length, parts, mcotxt::kMaxParts);
  if (part_count == 0 || part_index >= part_count) return 0;
  memcpy(output, header, header_length);
  const size_t length = mcotxt::formatPart(
      prefix, prefix_length, text, parts[part_index], part_index, part_count,
      (char*)output + header_length, MAX_FRAME_SIZE + 1 - header_length);
  return length == 0 || length > max_bytes ? 0 : (int)(header_length + length);
#else
  if (part_index != 0 || prefix_length + text_length > max_bytes) return 0;
  memcpy(output, header, header_length);
  if (prefix_length != 0) memcpy(output + header_length, prefix, prefix_length);
  memcpy(output + header_length + prefix_length, text, text_length);
  part_count = 1;
  return (int)(header_length + prefix_length + text_length);
#endif
}

int MyMesh::renderCompatFramePart(const Frame& source, size_t part_index,
                                  size_t& part_count, bool& additive,
                                  uint8_t output[]) {
  part_count = 0;
  additive = false;

  const uint8_t* const frame_bytes = frameData(source);
  if (!frame_bytes) return 0;

  CompatTextFrameLayout layout;
  if (compatTextFrameLayout(frame_bytes, source.len, layout)) {
#ifdef WITH_COMPANION_FRAME_FRAGMENTS
    char original[companion_frames::MAX_ORIGINAL_LEN + 1];
#else
    char original[MAX_FRAME_SIZE + 1];
#endif
    const size_t original_length = source.len - layout.header_length;
    if (original_length + 1 > sizeof(original)) return 0;
    memcpy(original, frame_bytes + layout.header_length, original_length);
    original[original_length] = '\0';

    mco_compat::Options options = { false, false, false };
#ifdef WITH_MCOTXT
    options.mcotxt = isMCOtxtEnabled() && !_app_supports_mctxt;
#endif
#ifdef WITH_MCMP_DETECT
    options.mcmp = isMCMPDetectEnabled() && !_app_supports_mcmp;
#endif
#ifdef WITH_MCOIMG_DETECT
    options.mcoimg = isMCOimgDetectEnabled() && !_app_supports_mcoimg;
#endif

#ifdef WITH_MCOTXT
    char* transformed = mcotxt::scratch();
    const size_t transformed_capacity = mcotxt::kScratchBytes;
#else
    char transformed[MAX_FRAME_SIZE + 1];
    const size_t transformed_capacity = sizeof(transformed);
#endif
    const size_t prefix_length = compatSenderPrefixLength(original, layout.has_sender_name);
    const mco_compat::Result result = transformCompatPayload(
        original, layout.has_sender_name, transformed, transformed_capacity, options);
    if (!result.changed) return 0;
    if (prefix_length > result.length) return 0;
    return formatCompatPart(frame_bytes, layout.header_length,
                            transformed, prefix_length,
                            transformed + prefix_length,
                            result.length - prefix_length,
                            part_index, part_count, output);
  }

  if (source.len < 9 || frame_bytes[0] != RESP_CODE_CHANNEL_DATA_RECV) return 0;
  const uint16_t data_type = (uint16_t)frame_bytes[6] | ((uint16_t)frame_bytes[7] << 8);
  const size_t data_length = frame_bytes[8];
  if (9 + data_length > source.len) return 0;
  const uint8_t* data = frame_bytes + 9;

  const char* body = nullptr;
  size_t body_length = 0;
  char prefix[40] = {};
  size_t prefix_length = 0;
  char placeholder[96] = {};
  uint32_t timestamp = _app_sync_timestamp;

#ifdef WITH_MCOTXT
  mcotxt::DecodedMessage mcotxt_message;
  char* decoded = mcotxt::scratch();
  if (isMCOtxtEnabled() && !_app_supports_mctxt &&
      mcotxt::isBinaryEnvelope(data_type, data, data_length)) {
    const mcotxt::MessageStatus status = mcotxt::decodeBinaryEnvelope(
        data_type, data, data_length, decoded, mcotxt::kScratchBytes, mcotxt_message);
    if (status == mcotxt::MessageStatus::Ok || status == mcotxt::MessageStatus::TooLong) {
      bool mention_truncated = false;
      if (mcotxt::ensureReplyMentionPrefix(mcotxt_message, decoded,
                                           mcotxt::kScratchBytes,
                                           mention_truncated)) {
        const int length = snprintf(prefix, sizeof(prefix), "%s: ",
                                    mcotxt_message.has_sender ? mcotxt_message.sender : "?");
        if (length > 0 && length < (int)sizeof(prefix)) {
          prefix_length = (size_t)length;
          body = decoded;
          body_length = strlen(decoded);
          if (mcotxt_message.has_timestamp) timestamp = mcotxt_message.timestamp;
        }
      }
    }
  }
#endif

#ifdef WITH_MCMP_DETECT
  if (body == nullptr && isMCMPDetectEnabled() && !_app_supports_mcmp) {
    mcmp::Meta meta;
    if (mcmp::parseBinaryEnvelope(data_type, data, data_length, meta)) {
      const int prefix_result = snprintf(prefix, sizeof(prefix), "%s: ",
                                         meta.has_sender ? meta.sender : "MCMP");
      const int body_result = mcmp::formatPlaceholder(
          meta, placeholder, sizeof(placeholder));
      if (prefix_result > 0 && prefix_result < (int)sizeof(prefix) &&
          body_result > 0 && body_result < (int)sizeof(placeholder)) {
        prefix_length = (size_t)prefix_result;
        body = placeholder;
        body_length = (size_t)body_result;
        if (meta.container_ok) timestamp = meta.timestamp;
      }
    }
  }
#endif

#ifdef WITH_AEIC_DETECT
  if (body == nullptr && isAEICDetectEnabled() && !_app_supports_aeic &&
      (source.compat_flags & FRAME_COMPAT_AEIC_NOTICE) != 0) {
    const aeic::ChunkInfo info = aeic::parseChunk(data_type, data, data_length);
    if (info.isData() || info.isParity()) {
      char sender[32];
      aeicSenderName(info.sender_prefix, sender, sizeof(sender));
      const int prefix_result = snprintf(prefix, sizeof(prefix), "%s: ", sender);
      if (prefix_result > 0 && prefix_result < (int)sizeof(prefix)) {
        prefix_length = (size_t)prefix_result;
        strcpy(placeholder, "<AEIC image>");
        body = placeholder;
        body_length = strlen(placeholder);
      }
    }
  }
#endif

#ifdef WITH_MCOIMG_DETECT
  if (body == nullptr && isMCOimgDetectEnabled() && !_app_supports_mcoimg) {
    mcoimg_detect::Meta meta;
    if (mcoimg_detect::parseBinaryEnvelope(data_type, data, data_length, meta)) {
      const int prefix_result = snprintf(prefix, sizeof(prefix), "%s: ",
                                         meta.sender[0] ? meta.sender : "MCOimg");
      const int body_result = snprintf(placeholder, sizeof(placeholder),
                                       "<MCOimg v%u image>", (unsigned)meta.version);
      if (prefix_result > 0 && prefix_result < (int)sizeof(prefix) &&
          body_result > 0 && body_result < (int)sizeof(placeholder)) {
        prefix_length = (size_t)prefix_result;
        body = placeholder;
        body_length = (size_t)body_result;
      }
    }
  }
#endif

  if (body == nullptr) return 0;
  uint8_t header[16];
  size_t header_length = 0;
  if (app_target_ver >= 3) {
    header[header_length++] = RESP_CODE_CHANNEL_MSG_RECV_V3;
    header[header_length++] = frame_bytes[1];
    header[header_length++] = 0;
    header[header_length++] = 0;
  } else {
    header[header_length++] = RESP_CODE_CHANNEL_MSG_RECV;
  }
  header[header_length++] = frame_bytes[4];  // channel index
  header[header_length++] = frame_bytes[5];  // packed path length
  header[header_length++] = TXT_TYPE_PLAIN;
  memcpy(header + header_length, &timestamp, sizeof(timestamp));
  header_length += sizeof(timestamp);
  additive = true;
  return formatCompatPart(header, header_length, prefix, prefix_length,
                          body, body_length, part_index, part_count, output);
}

#ifdef WITH_COMPANION_FRAME_FRAGMENTS
int MyMesh::getNextAppFrame(uint8_t frame[], bool has_ack, uint16_t ack_id,
                            uint8_t ack_index) {
  while (true) {
    if (!_app_sync_active) {
      if (offline_queue_len <= 0) return 0;
      _app_sync_active = true;
      _app_sync_original_sent = false;
      _app_sync_next_part = 0;
      _app_sync_timestamp = (uint32_t)getRTCClock()->getCurrentTime();
    }

    Frame& source = offline_queue[0];
    const uint8_t* source_data = frameData(source);
    if (!source_data) {
      bool ignored = false;
      emitQueuedOriginalForApp(frame, false, has_ack, ack_id, ack_index, ignored);
      continue;
    }
#if WITH_OFFLINE_ROUTES
    // The route goes out first, once per entry: a client binds a raw channel copy
    // to the message that arrives after it. Not before the check above — a route
    // pushed for an entry that is then dropped would bind to the next message.
    if (!_app_sync_route_sent) {
      _app_sync_route_sent = true;
      emitQueuedRoute(0);
    }
#endif
    size_t part_count = 0;
    bool additive = false;
    const int transformed_length = renderCompatFramePart(
        source, _app_sync_next_part, part_count, additive, frame);

    if (additive && !_app_sync_original_sent) {
      bool stage_complete = false;
      const int original_len = emitQueuedOriginalForApp(
          frame, true, has_ack, ack_id, ack_index, stage_complete);
      if (original_len > 0) return original_len;
      if (!stage_complete) return 0;
      has_ack = false;
      continue;
    }

    if (part_count == 0 || transformed_length <= 0) {
      if (_app_sync_original_sent) {
        removeOfflineQueueHead();
        resetFragmentTransferState();
        resetAppSyncState();
        continue;
      }
      bool stage_complete = false;
      const int original_len = emitQueuedOriginalForApp(
          frame, false, has_ack, ack_id, ack_index, stage_complete);
      if (original_len > 0) return original_len;
      if (!stage_complete) return 0;
      has_ack = false;
      continue;
    }

    _app_sync_next_part++;
    if (_app_sync_next_part >= part_count) {
      removeOfflineQueueHead();
      resetFragmentTransferState();
      resetAppSyncState();
    }
    return transformed_length;
  }
}
#else
int MyMesh::getNextAppFrame(uint8_t frame[]) {
  while (true) {
    if (!_app_sync_active) {
      if (offline_queue_len <= 0) return 0;
      _app_sync_active = true;
      _app_sync_original_sent = false;
      _app_sync_next_part = 0;
      _app_sync_timestamp = (uint32_t)getRTCClock()->getCurrentTime();
    }

    const Frame& source = offline_queue[0];
#if WITH_OFFLINE_ROUTES
    // The route goes out first, once per entry: a client binds a raw channel copy
    // to the message that arrives after it.
    if (!_app_sync_route_sent) {
      _app_sync_route_sent = true;
      emitQueuedRoute(0);
    }
#endif
    size_t part_count = 0;
    bool additive = false;
    const int transformed_length = renderCompatFramePart(
        source, _app_sync_next_part, part_count, additive, frame);

    if (additive && !_app_sync_original_sent) {
      memcpy(frame, source.buf, source.len);
      _app_sync_original_sent = true;
      return source.len;
    }

    if (part_count == 0 || transformed_length <= 0) {
      if (!_app_sync_original_sent) {
        memcpy(frame, source.buf, source.len);
        const int length = source.len;
        removeOfflineQueueHead();
        resetAppSyncState();
        return length;
      }
      removeOfflineQueueHead();
      resetAppSyncState();
      continue;
    }

    _app_sync_next_part++;
    if (_app_sync_next_part >= part_count) {
      removeOfflineQueueHead();
      resetAppSyncState();
    }
    return transformed_length;
  }
}
#endif

void MyMesh::setMCOtxtEnabled(bool enabled) {
#ifdef WITH_MCOTXT
  _prefs.ui_mcotxt_disabled = enabled ? 0 : 1;
  deferSavePrefs();
#endif
}

void MyMesh::setMCMPDetectEnabled(bool enabled) {
#ifdef WITH_MCMP_DETECT
  _prefs.ui_mcmp_detect_off = enabled ? 0 : 1;
  deferSavePrefs();
#endif
}

#ifdef WITH_AEIC_DETECT
bool MyMesh::markAEICNotice(uint8_t channel_idx, uint16_t sender_prefix,
                            uint8_t image_id, uint8_t total, uint32_t now_millis) {
  int replacement = -1;
  for (int i = 0; i < AEIC_NOTICE_TABLE_SIZE; ++i) {
    AEICNotice& notice = _aeic_notices[i];
    if (!notice.used) {
      if (replacement < 0) replacement = i;
      continue;
    }
    const uint32_t age = now_millis - notice.seen_at;
    if (notice.channel_idx == channel_idx && notice.sender_prefix == sender_prefix &&
        notice.image_id == image_id && notice.total == total &&
        age <= AEIC_NOTICE_TTL_MILLIS) return false;
    if (replacement < 0 && age > AEIC_NOTICE_TTL_MILLIS) replacement = i;
  }
  if (replacement < 0) {
    replacement = _next_aeic_notice;
    _next_aeic_notice = (_next_aeic_notice + 1) % AEIC_NOTICE_TABLE_SIZE;
  }
  AEICNotice& notice = _aeic_notices[replacement];
  notice = {true, channel_idx, sender_prefix, image_id, total, now_millis};
  return true;
}

void MyMesh::aeicSenderName(uint16_t sender_prefix, char* out, size_t out_size) {
  const uint8_t prefix[2] = {(uint8_t)(sender_prefix >> 8), (uint8_t)sender_prefix};
  if (memcmp(self_id.pub_key, prefix, sizeof(prefix)) == 0) {
    StrHelper::strzcpy(out, _prefs.node_name, out_size);
    return;
  }
  ContactInfo* contact = lookupContactByPubKey(prefix, sizeof(prefix));
  if (contact && contact->name[0]) {
    StrHelper::strzcpy(out, contact->name, out_size);
    return;
  }
  snprintf(out, out_size, "AEIC-%02X%02X", prefix[0], prefix[1]);
}

#endif

void MyMesh::setAEICDetectEnabled(bool enabled) {
#ifdef WITH_AEIC_DETECT
  _prefs.ui_aeic_detect_off = enabled ? 0 : 1;
  deferSavePrefs();
#endif
}

void MyMesh::setMCOimgDetectEnabled(bool enabled) {
#ifdef WITH_MCOIMG_DETECT
  _prefs.ui_mcoimg_detect_off = enabled ? 0 : 1;
  deferSavePrefs();
#endif
}

#ifdef WITH_COMPANION_FRAME_FRAGMENTS
void MyMesh::setFramedPacketsEnabled(bool enabled) {
  _prefs.framed_packets_enabled = enabled ? 1 : 0;
  if (!enabled) resetFragmentTransferState();
  deferSavePrefs();
}
#endif

#ifdef WITH_COMMS_SWITCHING

#ifdef ENABLE_WIFI_INTERFACE

void MyMesh::wifiPrefsFromNodePrefs() {
  memset(&_wifi_prefs, 0, sizeof(_wifi_prefs));
  _wifi_prefs.comms_mode = _prefs.comms_mode <= COMMS_MODE_USB ? _prefs.comms_mode : COMMS_MODE_BLE;
  int n = 0;
  if (_prefs.wifi_ssid[0]) {
    StrHelper::strncpy(_wifi_prefs.networks[n].ssid, _prefs.wifi_ssid, sizeof(_wifi_prefs.networks[n].ssid));
    StrHelper::strncpy(_wifi_prefs.networks[n].password, _prefs.wifi_pwd, sizeof(_wifi_prefs.networks[n].password));
    n++;
  }
  for (int i = 0; i < 4 && n < WIFI_MAX_NETWORKS; i++) {
    if (!_prefs.wifi_ssid_x[i][0]) continue;
    StrHelper::strncpy(_wifi_prefs.networks[n].ssid, _prefs.wifi_ssid_x[i], sizeof(_wifi_prefs.networks[n].ssid));
    StrHelper::strncpy(_wifi_prefs.networks[n].password, _prefs.wifi_pwd_x[i], sizeof(_wifi_prefs.networks[n].password));
    n++;
  }
  _wifi_prefs.network_count = n;
  _wifi_prefs.ip_mode = _prefs.wifi_ip_mode;
  StrHelper::strncpy(_wifi_prefs.static_ip, _prefs.wifi_ip, sizeof(_wifi_prefs.static_ip));
  StrHelper::strncpy(_wifi_prefs.static_gw, _prefs.wifi_gw, sizeof(_wifi_prefs.static_gw));
  StrHelper::strncpy(_wifi_prefs.static_mask, _prefs.wifi_mask, sizeof(_wifi_prefs.static_mask));
  _wifi_prefs.tcp_port = _prefs.wifi_port ? _prefs.wifi_port : WIFI_TCP_PORT_DEFAULT;
}

void MyMesh::loadWifiPrefs() {
  // Up to 1.17 the uni settings were a binary /wifi_prefs file: move them into
  // the prefs once and drop the file.
  File file = SPIFFS.open("/wifi_prefs", "r");
  if (file) {
    WifiPrefs old;
    memset(&old, 0, sizeof(old));
    file.read((uint8_t*)&old, min((size_t)file.size(), sizeof(old)));
    file.close();
    if (old.network_count > WIFI_MAX_NETWORKS) old.network_count = 0;
    memcpy(&_wifi_prefs, &old, sizeof(old));
    saveWifiPrefs();
    SPIFFS.remove("/wifi_prefs");
  }
  wifiPrefsFromNodePrefs();
}

#endif  // ENABLE_WIFI_INTERFACE

void MyMesh::saveWifiPrefs() {
  _prefs.comms_mode = _wifi_prefs.comms_mode;
#ifdef ENABLE_WIFI_INTERFACE
  const WifiNetwork* nets = _wifi_prefs.networks;
  const int n = _wifi_prefs.network_count;
  StrHelper::strncpy(_prefs.wifi_ssid, n > 0 ? nets[0].ssid : "", sizeof(_prefs.wifi_ssid));
  StrHelper::strncpy(_prefs.wifi_pwd, n > 0 ? nets[0].password : "", sizeof(_prefs.wifi_pwd));
  for (int i = 0; i < 4; i++) {
    StrHelper::strncpy(_prefs.wifi_ssid_x[i], i + 1 < n ? nets[i + 1].ssid : "", sizeof(_prefs.wifi_ssid_x[i]));
    StrHelper::strncpy(_prefs.wifi_pwd_x[i], i + 1 < n ? nets[i + 1].password : "", sizeof(_prefs.wifi_pwd_x[i]));
  }
  _prefs.wifi_enabled = _wifi_prefs.comms_mode == COMMS_MODE_WIFI ? 1 : 0;
  _prefs.wifi_ip_mode = _wifi_prefs.ip_mode;
  StrHelper::strncpy(_prefs.wifi_ip, _wifi_prefs.static_ip, sizeof(_prefs.wifi_ip));
  StrHelper::strncpy(_prefs.wifi_gw, _wifi_prefs.static_gw, sizeof(_prefs.wifi_gw));
  StrHelper::strncpy(_prefs.wifi_mask, _wifi_prefs.static_mask, sizeof(_prefs.wifi_mask));
  _prefs.wifi_port = _wifi_prefs.tcp_port == WIFI_TCP_PORT_DEFAULT ? 0 : _wifi_prefs.tcp_port;
#endif
  savePrefs();
}

#ifdef ENABLE_WIFI_INTERFACE

void MyMesh::addWifiNetwork(const char* ssid, const char* pass) {
  // Update existing entry if SSID already stored
  for (int i = 0; i < _wifi_prefs.network_count; i++) {
    if (strcmp(_wifi_prefs.networks[i].ssid, ssid) == 0) {
      strncpy(_wifi_prefs.networks[i].password, pass, sizeof(_wifi_prefs.networks[i].password) - 1);
      saveWifiPrefs();
      return;
    }
  }
  if (_wifi_prefs.network_count >= WIFI_MAX_NETWORKS) return;  // full
  int idx = _wifi_prefs.network_count++;
  strncpy(_wifi_prefs.networks[idx].ssid,     ssid, sizeof(_wifi_prefs.networks[idx].ssid) - 1);
  strncpy(_wifi_prefs.networks[idx].password, pass, sizeof(_wifi_prefs.networks[idx].password) - 1);
  saveWifiPrefs();
}

void MyMesh::removeWifiNetwork(const char* ssid) {
  for (int i = 0; i < _wifi_prefs.network_count; i++) {
    if (strcmp(_wifi_prefs.networks[i].ssid, ssid) == 0) {
      // Shift remaining entries left
      for (int j = i; j < _wifi_prefs.network_count - 1; j++)
        _wifi_prefs.networks[j] = _wifi_prefs.networks[j + 1];
      memset(&_wifi_prefs.networks[--_wifi_prefs.network_count], 0, sizeof(WifiNetwork));
      saveWifiPrefs();
      return;
    }
  }
}

#endif  // ENABLE_WIFI_INTERFACE

void MyMesh::switchCommsMode(uint8_t mode, int wifi_net_idx) {
#ifdef WITH_COMPANION_FRAME_FRAGMENTS
  resetFragmentSession();
#endif
  if (mode > COMMS_MODE_USB) mode = COMMS_MODE_BLE;
  if (!commsModeAvailable(mode)) mode = COMMS_MODE_BLE;
#ifdef ENABLE_WIFI_INTERFACE
  if (mode == COMMS_MODE_WIFI && _wifi_prefs.network_count == 0) mode = COMMS_MODE_BLE;
  // The chosen network goes first: that is the one the transport joins.
  if (mode == COMMS_MODE_WIFI && wifi_net_idx > 0 && wifi_net_idx < _wifi_prefs.network_count) {
    WifiNetwork chosen = _wifi_prefs.networks[wifi_net_idx];
    for (int i = wifi_net_idx; i > 0; i--) _wifi_prefs.networks[i] = _wifi_prefs.networks[i - 1];
    _wifi_prefs.networks[0] = chosen;
  }
#else
  (void)wifi_net_idx;
#endif
  _wifi_prefs.comms_mode = mode;
  saveWifiPrefs();
  _wifi_attempt_start = millis();
  if (_comms_hook) _comms_hook(mode);
}

#ifdef ENABLE_WIFI_INTERFACE

// Upstream's wifi.* setters, on the uni model: network 1 is upstream's
// wifi.ssid / wifi.pwd, and WiFi runs when the transport mode says so. Here, not
// in runCliChain, so the app's CLI frame and the rescue CLI get it too.
bool MyMesh::handleUniWifiSet(const char* command, char* reply) {
  WifiNetwork* n0 = &_wifi_prefs.networks[0];
  if (memcmp(command, "set wifi.ssid ", 14) == 0) {
    if (_wifi_prefs.network_count == 0) {
      memset(n0, 0, sizeof(*n0));
      _wifi_prefs.network_count = 1;
    }
    StrHelper::strncpy(n0->ssid, &command[14], sizeof(n0->ssid));
    saveWifiPrefs();
    sprintf(reply, "> wifi.ssid is now %s (set wifi.pwd too)", n0->ssid);
    return true;
  }
  if (memcmp(command, "set wifi.pwd ", 13) == 0) {
    if (_wifi_prefs.network_count == 0) {
      strcpy(reply, "Error: set wifi.ssid first");
      return true;
    }
    StrHelper::strncpy(n0->password, &command[13], sizeof(n0->password));
    saveWifiPrefs();
    if (_wifi_prefs.comms_mode == COMMS_MODE_WIFI) switchCommsMode(COMMS_MODE_WIFI, 0);   // rejoin
    strcpy(reply, "> wifi.pwd updated");
    return true;
  }
  if (strcmp(command, "set wifi.clear") == 0) {
    if (_wifi_prefs.network_count > 0) removeWifiNetwork(n0->ssid);
    if (_wifi_prefs.network_count == 0 && _wifi_prefs.comms_mode == COMMS_MODE_WIFI) {
      switchCommsMode(COMMS_MODE_BLE);
    }
    strcpy(reply, "> wifi network 1 cleared");
    return true;
  }
  if (memcmp(command, "set wifi.enabled ", 17) == 0) {
    bool on = atoi(&command[17]) != 0;
    if (on && _wifi_prefs.network_count == 0) {
      strcpy(reply, "Error: no network, set wifi.ssid first");
      return true;
    }
    switchCommsMode(on ? COMMS_MODE_WIFI : COMMS_MODE_BLE);
    sprintf(reply, "> wifi.enabled is now %d", on ? 1 : 0);
    return true;
  }
  return false;
}

void MyMesh::wifiScanDone() {
  WiFi.scanDelete();
  if (_wifi_prefs.comms_mode != COMMS_MODE_WIFI) WiFi.mode(WIFI_OFF);
}

#endif  // ENABLE_WIFI_INTERFACE

void MyMesh::initCommsFromPrefs() {
#ifdef ENABLE_WIFI_INTERFACE
  loadWifiPrefs();
#else
  // No WiFi interface: only BLE and USB can be selected, and a stored WiFi
  // mode is applied as BLE.
  memset(&_wifi_prefs, 0, sizeof(_wifi_prefs));
  _wifi_prefs.comms_mode = commsModeAvailable(_prefs.comms_mode) ? _prefs.comms_mode : COMMS_MODE_BLE;
#endif
  if (_active_ble_pin == 0) {
    if (_prefs.ble_pin != 0) {
      _active_ble_pin = _prefs.ble_pin;  // restore persisted pin (BLE_PIN_CODE not set in uni builds)
    } else {
      StdRNG rng;
      _active_ble_pin = rng.nextInt(100000, 999999);
      // auto mode: new random each boot, not persisted
    }
  }
  _wifi_attempt_start = millis();
}

#endif  // WITH_COMMS_SWITCHING

// To check if there is pending work
bool MyMesh::hasPendingWork() const {
  return _mgr->getOutboundTotal() > 0 || dirty_contacts_expiry != 0;
}
