// BTstack configuration for the Pico 2 W hexapod firmware (plan part 02).
//
// BT Classic + BLE HID host — the profile Bluepad32 needs to pair PS4/PS5 and
// Xbox pads over the CYW43439. Derived from the canonical
// bluepad32 / pico-examples `btstack_config.h` for Pico W. If pairing regresses
// after bumping the vendored Bluepad32 or Pico SDK BTstack, re-diff against that
// upstream file — the memory/flow-control tuning below is what keeps the
// shared-bus CYW43 transport from overrunning.
#ifndef BTSTACK_CONFIG_H
#define BTSTACK_CONFIG_H

// ── Port / HAL features ────────────────────────────────────────────────────
#define HAVE_ASSERT
#define HAVE_EMBEDDED_TIME_MS
// Some controllers are slow to answer the initial HCI reset.
#define HCI_RESET_RESEND_TIMEOUT_MS 1000

// ── Enabled BTstack features ───────────────────────────────────────────────
#define ENABLE_CLASSIC
#define ENABLE_BLE
#define ENABLE_LE_CENTRAL
#define ENABLE_LE_PERIPHERAL
#define ENABLE_LE_SECURE_CONNECTIONS
#define ENABLE_LE_DATA_LENGTH_EXTENSION
#define ENABLE_L2CAP_ENHANCED_RETRANSMISSION_MODE
#define ENABLE_GATT_OVER_CLASSIC
#define ENABLE_SOFTWARE_AES128
#define ENABLE_MICRO_ECC_FOR_LE_SECURE_CONNECTIONS
#define ENABLE_LOG_ERROR
#define ENABLE_LOG_INFO
// Required by btstack's hci_dump_embedded_stdout.c, which the Pico SDK btstack
// build (pulled in by Bluepad32) compiles unconditionally.
#define ENABLE_PRINTF_HEXDUMP

// ── Buffers / sizes ────────────────────────────────────────────────────────
#define HCI_OUTGOING_PRE_BUFFER_SIZE 4
#define HCI_ACL_PAYLOAD_SIZE (1691 + 4)
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT 4
#define MAX_NR_AVDTP_CONNECTIONS 0
#define MAX_NR_AVDTP_STREAM_ENDPOINTS 0
#define MAX_NR_AVRCP_CONNECTIONS 0
#define MAX_NR_BNEP_CHANNELS 0
#define MAX_NR_BNEP_SERVICES 0
#define MAX_NR_BTSTACK_LINK_KEY_DB_MEMORY_ENTRIES 2
#define MAX_NR_GATT_CLIENTS 1
#define MAX_NR_HCI_CONNECTIONS 4
#define MAX_NR_HID_HOST_CONNECTIONS 4
#define MAX_NR_HIDS_CLIENTS 4
#define MAX_NR_L2CAP_CHANNELS 8
#define MAX_NR_L2CAP_SERVICES 4
#define MAX_NR_RFCOMM_CHANNELS 0
#define MAX_NR_RFCOMM_MULTIPLEXERS 0
#define MAX_NR_RFCOMM_SERVICES 0
#define MAX_NR_SERVICE_RECORD_ITEMS 4
#define MAX_NR_SM_LOOKUP_ENTRIES 3
#define MAX_NR_WHITELIST_ENTRIES 16
#define MAX_NR_LE_DEVICE_DB_ENTRIES 16

// Limit ACL/SCO buffers so the CYW43 shared SPI bus can't overrun, and turn on
// HCI controller-to-host flow control for the same reason.
#define MAX_NR_CONTROLLER_ACL_BUFFERS 3
#define MAX_NR_CONTROLLER_SCO_PACKETS 3
#define ENABLE_HCI_CONTROLLER_TO_HOST_FLOW_CONTROL
#define HCI_HOST_ACL_PACKET_LEN 1024
#define HCI_HOST_ACL_PACKET_NUM 3
#define HCI_HOST_SCO_PACKET_LEN 120
#define HCI_HOST_SCO_PACKET_NUM 3

// ── Persistent storage (link keys / LE device DB) via flash-sector TLV ──────
#define NVM_NUM_DEVICE_DB_ENTRIES 16
#define NVM_NUM_LINK_KEYS 16

// Fixed-size ATT DB (no malloc for the DB).
#define MAX_ATT_DB_SIZE 512

#endif  // BTSTACK_CONFIG_H
