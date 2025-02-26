#ifndef __NORDIC_52832_NFC_
#define __NORDIC_52832_NFC_

enum
{
    NFCSTATE_IDLE,
    NFCSTATE_READ_INFO,
    NFCSTATE_READ_DATA,
};

extern uint8_t data_recived_buf[APDU_BUFF_SIZE];
extern uint16_t data_recived_len;

int nfc_init(void);
void nfc_poll(void* p_event_data, uint16_t event_size);
#endif
