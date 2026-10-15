#include "Logger.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

void Logger::begin() {
    head_ = 0;
    tail_ = 0;
}

void Logger::printf(const char* fmt, ...) {
    const size_t next = (head_ + 1) % RING_SIZE;
    if (next == tail_) return;   // ring full, drop

    char buf[MAX_LINE];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, MAX_LINE, fmt, ap);
    va_end(ap);

    memcpy(ring_[head_], buf, MAX_LINE);
    head_ = next;
}

bool Logger::pop(char* out) {
    if (head_ == tail_) return false;
    memcpy(out, ring_[tail_], MAX_LINE);
    tail_ = (tail_ + 1) % RING_SIZE;
    return true;
}