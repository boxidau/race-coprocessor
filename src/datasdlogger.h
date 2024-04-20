#include "Arduino.h"
#include "stringformat.h"

#define FLUSH_MS 0

class DataSDLogger
{
    public:
        static void setup();
        static void logComment(const String line);
        static bool logData(const char* data, size_t len);
        static bool logData(const char* data);
};