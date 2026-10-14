#include <Arduino.h>
#include "bhi385_app.h"

void setup()
{
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    delay(500);
    bhi385_app_init();
}

void loop()
{
    bhi385_app_loop();
    // delay(10);
}