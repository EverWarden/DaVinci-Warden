#include "utils.h"
#include <SCServo.h>

void debugPrint(int level, const char* format, ...)
{
    if (level > DEBUG_LEVEL)
        return;

    char buffer[256];

    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    Serial.print(buffer);
}