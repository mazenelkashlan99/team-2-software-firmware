#include "log_task.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdarg.h>

#include "config.h"
#include "Logger.h"

namespace {

Logger s_logger;
constexpr size_t LOG_LINE_MAX = 128;

struct LogLine {
    char text[LOG_LINE_MAX];
};

QueueHandle_t s_logQ = nullptr;

}  // namespace

void logInit() {
    s_logger.begin();
    s_logQ = xQueueCreate(LOG_QUEUE_LEN, sizeof(LogLine));
}

void logPrintf(const char* fmt, ...) {
    if (s_logQ == nullptr) return;

    LogLine line {};
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line.text, LOG_LINE_MAX, fmt, ap);
    va_end(ap);

    // Drop rather than block: logging must never stall a producer.
    xQueueSend(s_logQ, &line, 0);
}

void logTask(void*) {
    TickType_t wake = xTaskGetTickCount();
    LogLine line {};

    for (;;) {
        // Drain everything currently queued.
        while (xQueueReceive(s_logQ, &line, 0) == pdTRUE) {
            Serial.println(line.text);
        }

        // If you also want network logging, do that here via the
        // Logger class. Keep it off the hot path — never block on
        // WiFi from this task.

        vTaskDelayUntil(&wake, pdMS_TO_TICKS(LOG_PERIOD_MS));
    }
}