
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/services/nus.h>
#include <zephyr/logging/log.h>
#include <shell/shell_bt_nus.h>
#include <stdio.h>
#include "mic.h"

LOG_MODULE_REGISTER(ble_shell);

#define TEST_PACKET_SIZE 50
#define READER_STACK_SIZE 2048*10
#define READER_PRIORITY K_PRIO_PREEMPT(6)

K_THREAD_STACK_DEFINE(reader_stack_area, READER_STACK_SIZE);
static struct k_thread reader_thread_data;

uint8_t audio_ring_buffer_data[BLOCK_SIZE]; // 2 bytes per sample
struct ring_buf audio_ring_buf;

#define BT_UUID_OMI_VAL \
	BT_UUID_128_ENCODE(0x19b10000, 0xe8f2, 0x537e, 0x4f6c, 0xd104768a1214)

#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	        (sizeof(DEVICE_NAME) - 1)


static struct bt_conn *current_conn;
static volatile bool audio_subscribed = false;

// Use the same UUIDs as transport.c for compatibility with clients
static struct bt_uuid_128 audio_service_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x19B10000, 0xE8F2, 0x537E, 0x4F6C, 0xD104768A1214));
static struct bt_uuid_128 audio_characteristic_data_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x19B10001, 0xE8F2, 0x537E, 0x4F6C, 0xD104768A1214));

static void audio_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    audio_subscribed = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Client %s", audio_subscribed ? "subscribed" : "unsubscribed");
}

// Minimal GATT service definition for the test
static struct bt_gatt_attr audio_service_attrs[] = {
    BT_GATT_PRIMARY_SERVICE(&audio_service_uuid),
    BT_GATT_CHARACTERISTIC(&audio_characteristic_data_uuid.uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, NULL, NULL, NULL),
    BT_GATT_CCC(audio_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
};

static struct bt_gatt_service audio_service = BT_GATT_SERVICE(audio_service_attrs);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_OMI_VAL),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("Connection failed, err 0x%02x %s", err, bt_hci_err_to_str(err));
		return;
	}

	LOG_INF("Connected");
	current_conn = bt_conn_ref(conn);
	shell_bt_nus_enable(conn);
	audio_subscribed = false;
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected, reason 0x%02x %s", reason, bt_hci_err_to_str(reason));

	shell_bt_nus_disable();
	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
		audio_subscribed = false;
	}
}

static char *log_addr(struct bt_conn *conn)
{
	static char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	return addr;
}

static void __attribute__((unused)) security_changed(struct bt_conn *conn,
						     bt_security_t level,
						     enum bt_security_err err)
{
	char *addr = log_addr(conn);

	if (!err) {
		LOG_INF("Security changed: %s level %u", addr, level);
	} else {
		LOG_INF("Security failed: %s level %u err %d %s", addr, level, err,
			bt_security_err_to_str(err));
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
	COND_CODE_1(CONFIG_BT_SMP,
		    (.security_changed = security_changed), ())
};

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	LOG_INF("Passkey for %s: %06u", log_addr(conn), passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	LOG_INF("Pairing cancelled: %s", log_addr(conn));
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	LOG_INF("Pairing completed: %s, bonded: %d", log_addr(conn), bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	LOG_INF("Pairing failed conn: %s, reason %d %s", log_addr(conn), reason,
		bt_security_err_to_str(reason));
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};


static void reader_notifier_thread_entry(void *p1, void *p2, void *p3)
{
    uint8_t data_buffer[TEST_PACKET_SIZE];
    int err;

    LOG_INF("Reader/Notifier thread started");

    while (1) {
        // Wait until connected and subscribed
        while (!current_conn || !audio_subscribed) {
            k_msleep(100); // Check periodically
        }

        // Read data from ring buffer
        int read = ring_buf_get(&audio_ring_buf, data_buffer, TEST_PACKET_SIZE);

        if (read == TEST_PACKET_SIZE) {
            // Send notification
            err = bt_gatt_notify(current_conn, &audio_service.attrs[1], data_buffer, TEST_PACKET_SIZE);
            if (err == -EAGAIN || err == -ENOMEM) {
                // Queue is full, retry after a short delay
                LOG_WRN("bt_gatt_notify failed (%d), retrying...", err);
                // Put data back into ring buffer (might fail if buffer became full)
                if (ring_buf_put(&audio_ring_buf, data_buffer, TEST_PACKET_SIZE) != TEST_PACKET_SIZE) {
                     LOG_ERR("Failed to put data back into ring buffer after notify failure!");
                }
                k_msleep(5); // Small delay before retrying
                continue; // Skip yield at the end, try again immediately
            }
        } else if (read == 0) {
            // Buffer is empty, wait a bit
            k_msleep(5);
        } else {
             // Should not happen if writes are always TEST_PACKET_SIZE
             LOG_ERR("Ring buffer read unexpected size: %d", read);
        }

        // Yield to other threads
        k_yield();
    }
}

#define READER_STACK_SIZE 2048*10
#define READER_THREAD_PRIORITY 5
K_THREAD_DEFINE(reader_thread_id, READER_STACK_SIZE, reader_thread_function,
                NULL, NULL, NULL, READER_THREAD_PRIORITY, 0, -1);

static int cmd_ble_on(void)
{
	int err;

	printk("Starting Bluetooth NUS shell transport example\n");

	if (IS_ENABLED(CONFIG_BT_SMP)) {
		err = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (err) {
			printk("Failed to register authorization callbacks.\n");
			return 0;
		}

		err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (err) {
			printk("Failed to register authorization info callbacks.\n");
			return 0;
		}
	}

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("BLE enable failed (err: %d)", err);
		return 0;
	}

	err = shell_bt_nus_init();
	if (err) {
		LOG_ERR("Failed to initialize BT NUS shell (err: %d)", err);
		return 0;
	}

    // 2. Register audio GATT Service
    err = bt_gatt_service_register(&audio_service);
     if (err) {
        LOG_ERR("Failed to register test GATT service (err %d)", err);
        return err;
    }
    LOG_INF("Test GATT service registered");

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd,
			      ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return 0;
	}

	k_thread_start(reader_thread_id);
	LOG_INF("Bluetooth ready. Advertising started.");

	return 0;
}

static int cmd_ble_off(void)
{
    int err;
    // unregister authorization callbacks
    if (IS_ENABLED(CONFIG_BT_SMP)) {
        bt_conn_auth_cb_register(NULL);
        bt_conn_auth_info_cb_unregister(&conn_auth_info_callbacks);
    }
	err = bt_disable();
	if (err < 0) {
		printk("Bluetooth disable failed (%d)", err);
		return err;
	}
	k_thread_abort(reader_thread_id);
    printk("Bluetooth disabled");
    return 0;
}


SHELL_STATIC_SUBCMD_SET_CREATE(sub_ble_cmds,
                               SHELL_CMD(on, NULL, "Turn on BLE", cmd_ble_on),
                               SHELL_CMD(off, NULL, "Turn off BLE", cmd_ble_off),
                               SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(ble, &sub_ble_cmds, "BLE control commands", NULL);

static void audio_handler(int16_t *buffer, size_t size)
{
	// Check if the ring buffer has enough space
	if (ring_buf_space_get(&audio_ring_buf) < size)
	{
		LOG_ERR("Not enough space in ring buffer");
		return;
	}

	// Write the audio data to the ring buffer
	int written = ring_buf_put(&audio_ring_buf, (uint8_t *)buffer, size);
	if (written != size)
	{
		LOG_ERR("Failed to write to ring buffer: %d", written);
		return;
	}

	// Process the audio data
}

int bt_init(void)
{
	ring_buf_init(&audio_ring_buf, sizeof(audio_ring_buffer_data), audio_ring_buffer_data);
	set_mic_callback(audio_handler);
	return 0;
}