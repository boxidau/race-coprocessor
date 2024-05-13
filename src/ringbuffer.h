#include <Arduino.h>

template <typename T, size_t N> class RingBuffer {
    public:
        void push(T value) {
            buffer[idx] = value;
            if (++idx == N) {
                idx = 0;
                full = true;
            }
        }

        size_t size() const {
            return full ? N : idx;
        }

        bool isFull() const {
            return full;
        }

        bool hasSampleAt(int32_t offset) const {
            return offset > 0 ? full : (int32_t) size() > offset;
        }

        // assumes the buffer has enough samples for the given offset.
        // use hasSampleAt() to verify.
        const T& operator[](int32_t offset) const {
            int32_t index = idx + offset - 1;
            if (index < 0) {
                index += N;
            }
            return buffer[index];
        }

    private:
        T buffer[N];
        size_t idx { 0 };
        bool full { false };
};
