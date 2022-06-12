#include <Arduino.h>
#include <DaliWatchy.h>

#include "settings.h"

DaliWatchy watchy(settings);

void setup() {
  watchy.init();
}

void loop() {
  // put your main code here, to run repeatedly:
}