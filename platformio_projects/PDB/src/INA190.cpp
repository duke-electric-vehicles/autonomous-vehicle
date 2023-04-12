#include "INA190.hpp"

INA190::INA190(int pin) {
    this->pin = pin;
}

float INA190::getCurrent() {
    int analogIn = analogRead(pin);
    float shuntVoltage = analogIn * (3.3 / 1023.0);// * (1/SENSE_GAIN);
    float current = shuntVoltage * 10;// / R_SHUNT ;
    return current;
}
