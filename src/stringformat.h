#pragma once

#include <string.h>

#define INT_TO_CHAR(in) (0x30 + (in))
#define MAX_INT32_CHARS 11
#define MAX_FLOAT_3DP_CHARS (MAX_INT32_CHARS + 4)
#define REMAIN_PAD 3

template <size_t Size, char Delim = '\0'> class StringFormat {
    private:
        char str[Size];
        char* next;
        bool begin;

    public:
        StringFormat()
            : next(str)
            , begin(true)
        {
        }

        void formatFloat3DP(float in) {
            if (!checkLenAndWriteDelim(MAX_FLOAT_3DP_CHARS)) {
                return;
            }
            next = d3toa(next, in);
        }

        void formatUnsignedInt(uint32_t in) {
            if (!checkLenAndWriteDelim(MAX_INT32_CHARS)) {
                return;
            }
            next = uitoa(next, in);
        }

        void formatInt(int32_t in) {
            if (!checkLenAndWriteDelim(MAX_INT32_CHARS)) {
                return;
            }
            next = itoa(next, in);
        }

        void formatString(const char* in) {
            size_t len = strlen(in);
            if (!checkLenAndWriteDelim(len)) {
                return;
            }
            strcpy(next, in);
            next += len;
        }

        void formatString(const char* in, size_t len) {
            if (!checkLenAndWriteDelim(len)) {
                return;
            }
            strncpy(next, in, len);
            next += len;
        }

        template <size_t N> void formatLiteral(const char (&in)[N]) {
            size_t len = N - 1;
            if (!checkLenAndWriteDelim(len)) {
                return;
            }
            strcpy(next, in);
            next += len;
        }

        void formatBool(bool in) {
            if (!checkLenAndWriteDelim(1)) {
                return;
            }
            next = booltoa(next, in);
        }

        void formatBinary(char in) {
            if (!checkLenAndWriteDelim(8)) {
                return;
            }
            next = btoa(next, in);
        }

        char* finish() {
            *next++ = '\n';
            *next = '\0';
            return str;
        }

        uint32_t length() {
            return next - str;
        }

    private:
        bool checkLenAndWriteDelim(int max) {
            if ((int32_t) Size - REMAIN_PAD - (next - str) < max) {
                return false;
            }

            if (Delim) {
                if (!begin) {
                    *next++ = Delim;
                }
                begin = false;
            }
            return true;
        };

        static char* uitoa(char* out, uint32_t i) {
            uint32_t d = i, extraDigits = 0;
            while (d /= 10) {
                extraDigits++;
            }

            char* reverse = out + extraDigits;
            do {
                *reverse-- = INT_TO_CHAR(i % 10);
                i = i / 10;
            } while (i != 0);

            return out + extraDigits + 1;
        }

        static char* itoa(char* out, int32_t i) {
            if (i < 0) {
                *out++ = '-';
            }
            return uitoa(out, abs(i));
        }

        static char* booltoa(char* out, bool in) {
            *out++ = INT_TO_CHAR(!!in);
            return out;
        }

        static char* btoa(char* out, byte in) {
            // skip leading zeroes
            uint32_t bits = 0;
            while (!(in & 0x80) && bits < 7) {
                in <<= 1;
                bits++;
            }
            while (bits++ < 8) {
                *out++ = INT_TO_CHAR(!!(in & 0x80));
                in <<= 1;
            }
            return out;
        }

        static char* d3toa(char* out, float in) {
            int32_t integer1000 = roundf(in * 1000);
            if (integer1000 < 0) {
                *out++ = '-';
                integer1000 = -integer1000;
            }
            out = uitoa(out, integer1000 / 1000);
            *out++ = '.';
            *out++ = INT_TO_CHAR((integer1000 / 100) % 10);
            *out++ = INT_TO_CHAR((integer1000 / 10) % 10);
            *out++ = INT_TO_CHAR(integer1000 % 10);
            return out;
        }
};
