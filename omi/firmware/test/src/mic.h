#ifndef APP_SRC_MIC_H_
#define APP_SRC_MIC_H_

#define BITS_PER_BYTE 8

#define SAMPLE_RATE_HZ 16000
#define SAMPLE_BITS 16
#define CHANNEL_COUNT 2
#define TIMEOUT_MS 2000
#define CAPTURE_MS 100
#define BLOCK_SIZE ((SAMPLE_BITS / BITS_PER_BYTE) * (SAMPLE_RATE_HZ * CAPTURE_MS) / 1000) * CHANNEL_COUNT
#define BLOCK_COUNT 2

typedef void (*mix_handler)(int16_t *, size_t);

int mic_init(void);
void set_mic_callback(mix_handler callback);

#endif /* APP_SRC_MIC_H_ */
