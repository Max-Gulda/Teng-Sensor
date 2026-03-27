#ifndef ERROR_HANDLING_H
#define ERROR_HANDLING_H

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <zephyr/logging/log.h>

#define ERROR_CHECK(msg) \
    do { \
        if (errno != 0) { \
            LOG_ERR("ERROR: %s, failed with errno=%d (%s), in function: %s, in file: %s, on line: %d\n", \
                   msg, errno, strerror(errno), __FUNCTION__, __FILE__, __LINE__); \
        } \
    } while (0)

#endif 
