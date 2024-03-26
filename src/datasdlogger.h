#include "Arduino.h"
#include "stringformat.h"

#define PREALLOC_MB 1 // 1MB ~= 10 mins of logs, 18MB ~= 4 hours
#define FLUSH_MS 0

class DataSDLogger
{
    public:
        static void setup();
        static void logComment(const String line);
        static bool logData(const char* data, size_t len);
        static bool logData(const char* data);
};