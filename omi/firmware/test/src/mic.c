#include <stdio.h>
#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include "mic.h"

LOG_MODULE_REGISTER(mic, CONFIG_LOG_DEFAULT_LEVEL);

static const struct device *const dmic = DEVICE_DT_GET(DT_ALIAS(dmic0));
static const struct gpio_dt_spec mic_en = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(pdm_en_pin), gpios, {0});
static const struct gpio_dt_spec mic_thsel = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(pdm_thsel_pin), gpios, {0});
static const struct gpio_dt_spec mic_wake = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(pdm_wake_pin), gpios, {0});

K_MEM_SLAB_DEFINE_STATIC(mem_slab, BLOCK_SIZE, BLOCK_COUNT, 4);

static struct pcm_stream_cfg stream = {
	.pcm_rate = SAMPLE_RATE_HZ,
	.pcm_width = SAMPLE_BITS,
	.block_size = BLOCK_SIZE,
	.mem_slab = &mem_slab,
};

static struct dmic_cfg cfg = {
	.io =
		{
			.min_pdm_clk_freq = 1000000,
			.max_pdm_clk_freq = 3500000,
			.min_pdm_clk_dc = 40,
			.max_pdm_clk_dc = 60,
		},
	.streams = &stream,
	.channel =
		{
			.req_num_streams = 1,
			.req_num_chan = CHANNEL_COUNT,
		},
};


static volatile bool mic_running = false;
static volatile mix_handler callback_func = NULL;

static void mic_thread_function(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
	int written;

    while (mic_running) {
        void *buffer;
        uint32_t size;

        int ret = dmic_read(dmic, 0, &buffer, &size, TIMEOUT_MS);
        if (ret < 0) {
            LOG_ERR("Read failed: %d", ret);
            continue;
        }

        LOG_DBG("Got buffer %p of %u bytes", buffer, size);
		if (callback_func) {
        	callback_func((int16_t *)buffer, size);
    	}
		if (written != size * 2)
		{
			LOG_ERR("Failed to write %d bytes to codec ring buffer", size * 2);
			return -1;
		}
		k_mem_slab_free(&mem_slab, buffer);
    }
}

#define MIC_THREAD_STACK_SIZE 2048
#define MIC_THREAD_PRIORITY 5
K_THREAD_DEFINE(mic_thread_id, MIC_THREAD_STACK_SIZE, mic_thread_function,
                NULL, NULL, NULL, MIC_THREAD_PRIORITY, 0, -1);


int mic_power_off(void)
{
	gpio_pin_configure_dt(&mic_thsel, GPIO_OUTPUT);
	gpio_pin_set_dt(&mic_thsel, 0);
	gpio_pin_configure_dt(&mic_wake, GPIO_INPUT);
	gpio_pin_configure_dt(&mic_en, GPIO_OUTPUT);
	gpio_pin_set(mic_en.port, mic_en.pin, 0);
	return 0;
}

int mic_power_on(void)
{
	gpio_pin_configure_dt(&mic_thsel, GPIO_OUTPUT);
	gpio_pin_set_dt(&mic_thsel, 1);
	gpio_pin_configure_dt(&mic_wake, GPIO_INPUT);
	gpio_pin_configure_dt(&mic_en, GPIO_OUTPUT);
	gpio_pin_set_dt(&mic_en, 1);
	return 0;
}

static void mic_off(const struct shell *sh, size_t argc, char **argv)
{
	mic_power_off();
    if (mic_running) {
        mic_running = false;
        k_thread_abort(mic_thread_id);
        
        int ret = dmic_trigger(dmic, DMIC_TRIGGER_STOP);
        if (ret < 0) {
            LOG_ERR("STOP trigger failed: %d", ret);
        }
        
        LOG_INF("Microphone stopped");
    }
}

static void mic_on(const struct shell *sh, size_t argc, char **argv)
{
	mic_power_on();
    if (!mic_running) {
        int ret = dmic_trigger(dmic, DMIC_TRIGGER_START);
        if (ret < 0) {
            LOG_ERR("START trigger failed: %d", ret);
            return;
        }
        
        mic_running = true;
        k_thread_start(mic_thread_id);
        
        LOG_INF("Microphone restarted");
    }
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_mic_cmds,
							   SHELL_CMD(on, NULL, "microphone on", mic_on),
							   SHELL_CMD(off, NULL, "microphone off", mic_off),
							   SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(mic, &sub_mic_cmds, "Microphone", NULL);

int mic_init(void)
{
	if (!device_is_ready(dmic))
	{
		return -ENODEV;
	}

	mic_power_off();

	cfg.channel.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT) | 
	dmic_build_channel_map(1, 0, PDM_CHAN_RIGHT);

	return 0;
}

void set_mic_callback(mix_handler callback)
{
    callback_func = callback;
}