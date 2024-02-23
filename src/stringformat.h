#pragma once

#include <string.h>

#define INT_TO_CHAR(in) (0x30 + (in))
#define MAX_INT32_CHARS 11
#define MAX_FLOAT_3DP_CHARS (MAX_INT32_CHARS + 4)
#define REMAIN_PAD 3

class StringFormatCSV {
    private:
        char* str;
        char* next;
        uint32_t n;
        bool begin;

    public:
        StringFormatCSV(char* _str, uint32_t _n)
            : str(_str)
            , next(_str)
            , n(_n)
            , begin(true)
        {
        }

        void formatFloat3DP(float in) {
            if (!checkLenAndWriteComma(MAX_FLOAT_3DP_CHARS)) {
                return;
            }
            next = d3toa(next, in);
        }

        void formatUnsignedInt(uint32_t in) {
            if (!checkLenAndWriteComma(MAX_INT32_CHARS)) {
                return;
            }
            next = uitoa(next, in);
        }

        void formatInt(int32_t in) {
            if (!checkLenAndWriteComma(MAX_INT32_CHARS)) {
                return;
            }
            next = itoa(next, in);
        }

        void formatString(const char* in) {
            int len = strlen(in);
            if (!checkLenAndWriteComma(len)) {
                return;
            }
            strcpy(next, in);
            next += len;
        }

        void formatBool(bool in) {
            if (!checkLenAndWriteComma(1)) {
                return;
            }
            next = booltoa(next, in);
        }

        void formatBinary(char in) {
            if (!checkLenAndWriteComma(8)) {
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
        bool checkLenAndWriteComma(int max) {
            if ((int32_t) n - (next - str) < max + REMAIN_PAD) {
                return false;
            }
            if (!begin) {
                *next++ = ',';
            }
            begin = false;
            return true;
        };

        static char* uitoa(char* out, uint32_t i) {
            unsigned int d = i, extraDigits = 0;
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
            out = itoa(out, in);
            out = uitoa(out, round(abs(in) * 1000) % 1000 + 1000);
            out[-4] = '.';
            return out;
        }
};
