// RingBuffer.h
#pragma once
#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stddef.h>

class RingBuffer {
private:
    uint8_t* _buffer;
    size_t _size;
    volatile size_t _head;  // Read position
    volatile size_t _tail;  // Write position

public:
    RingBuffer(size_t size);
    ~RingBuffer();

    // Write single byte (for UART ISR)
    bool write(uint8_t data);

    // Write multiple bytes
    size_t write(const uint8_t* data, size_t len);

    // Read single byte
    bool read(uint8_t& data);

    // Peek at data without consuming (for frame detection)
    bool peek(size_t offset, uint8_t& data) const;

    // Peek at multiple bytes
    size_t peek(size_t offset, uint8_t* data, size_t len) const;

    // Skip/consume bytes without reading them
    size_t skip(size_t count);

    // Available data to read
    size_t available() const;

    // Free space for writing
    size_t free() const;

    // Check if buffer is full
    bool isFull() const;

    // Check if buffer is empty
    bool isEmpty() const;

    // Clear all data
    void clear();
};

#endif