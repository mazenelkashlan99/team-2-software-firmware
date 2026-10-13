#pragma once
#include <stddef.h>

// Small non-blocking ring-buffer logger. Producers format and push;
// a consumer task pops and writes to Serial or the network.
//
// The ring is full-checked, never blocks. Lines are dropped when full.
// Both printf() and pop() are safe to call from different tasks —
// head_ and tail_ are read/written as single words.
class Logger {
public:
    static constexpr size_t MAX_LINE  = 128;
    static constexpr size_t RING_SIZE = 32;

    void begin();

    // Format and enqueue. Drops the line if the ring is full.
    void printf(const char* fmt, ...)
        __attribute__((format(printf, 2, 3)));

    // Pop the oldest line into `out` (must be MAX_LINE bytes).
    // Returns false when empty.
    bool pop(char* out);

private:
    char     ring_[RING_SIZE][MAX_LINE];
    volatile size_t head_ = 0;
    volatile size_t tail_ = 0;
};