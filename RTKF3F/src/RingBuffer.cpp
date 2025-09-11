// RingBuffer.cpp
#include "RingBuffer.h"
#include <string.h>

RingBuffer::RingBuffer(size_t size) : _size(size), _head(0), _tail(0) {
    _buffer = new uint8_t[_size];
}

RingBuffer::~RingBuffer() {
    delete[] _buffer;
}

bool RingBuffer::write(uint8_t data) {
    size_t next_tail = (_tail + 1) % _size;
    if (next_tail == _head) {
        return false;  // Buffer full
    }
    _buffer[_tail] = data;
    _tail = next_tail;
    return true;
}

size_t RingBuffer::write(const uint8_t* data, size_t len) {
    size_t written = 0;
    for (size_t i = 0; i < len; i++) {
        if (write(data[i])) {
            written++;
        }
        else {
            break;  // Buffer full
        }
    }
    return written;
}

bool RingBuffer::read(uint8_t& data) {
    if (_head == _tail) {
        return false;  // Buffer empty
    }
    data = _buffer[_head];
    _head = (_head + 1) % _size;
    return true;
}

bool RingBuffer::peek(size_t offset, uint8_t& data) const {
    if (available() <= offset) {
        return false;
    }
    size_t pos = (_head + offset) % _size;
    data = _buffer[pos];
    return true;
}

size_t RingBuffer::peek(size_t offset, uint8_t* data, size_t len) const {
    size_t copied = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t byte;
        if (peek(offset + i, byte)) {
            data[i] = byte;
            copied++;
        }
        else {
            break;
        }
    }
    return copied;
}

size_t RingBuffer::skip(size_t count) {
    size_t skipped = 0;
    size_t avail = available();
    size_t toSkip = (count > avail) ? avail : count;

    _head = (_head + toSkip) % _size;
    return toSkip;
}

size_t RingBuffer::available() const {
    return (_tail - _head + _size) % _size;
}

size_t RingBuffer::free() const {
    return _size - available() - 1;  // -1 to distinguish full from empty
}

bool RingBuffer::isFull() const {
    return ((_tail + 1) % _size) == _head;
}

bool RingBuffer::isEmpty() const {
    return _head == _tail;
}

void RingBuffer::clear() {
    _head = _tail = 0;
}