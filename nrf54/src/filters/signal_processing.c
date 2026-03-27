#include "signal_processing.h"
#include "error_handling.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(signal_processing, LOG_LEVEL_INF);

int8_t sig_proc_derivative(int32_t signal_in[], int32_t signal_out[], uint16_t signal_length, uint8_t suppress_initial_transient) {

    if (signal_length <= 4 || !signal_in || !signal_out) {
        errno = EIO;
        ERROR_CHECK("Input signal error");
        return -1;
    }

    signal_out[0] = signal_in[0];
    signal_out[1] = signal_in[1];

    for (uint16_t i = 2; i < signal_length - 2; i++) {
        signal_out[i] = (-signal_in[i - 2]
            - (signal_in[i - 1] << 1)
            + (signal_in[i + 1] << 1)
            + signal_in[i + 2]) >> 3;
    }

    if (suppress_initial_transient > 0) {
        signal_out[signal_length - 2] = signal_out[signal_length - 3];
        signal_out[signal_length - 1] = signal_out[signal_length - 2];

        for (uint8_t i = 0; i < suppress_initial_transient; i++) {
            signal_out[i] = 0;
        }
    }
    return 0;
}


int8_t sig_proc_mw_integration(int32_t signal_in[], uint16_t signal_length, uint32_t win, uint32_t gain) {

    if (win == 0 || !signal_in) {
        errno = EIO;
        ERROR_CHECK("Input signal error");
        return -1;
    }

    int64_t acc = 0;
    uint32_t divider;

    for (uint32_t i = 0; i < signal_length; i++) {
        acc += (int64_t) signal_in[i];
        if (i >= win) {
            acc -= (int64_t) signal_in[i - win];
        }
        divider = (i + 1 < win) ? (i + 1) : win;

        signal_in[i] = (int32_t) (acc / (uint32_t) divider);

    }
    return 0;
}