#include <Arduino.h>

/**
 * Code for the TI-INA190 current sense amplifier IC.
 **/
class INA190 {
    private:
        static const int GAIN = 100; // in V/V
        int pin;
        int Rshunt;

    public:
        INA190(int pin, float shuntResistance){
            this->pin = pin;
            this->Rshunt = shuntResistance;
        }

        float getCurrent(){
            int analogIn = analogRead(pin);                     // read ADC
            float analogVoltage = analogIn * (3.3 / 1023.0);      // ADC counts to V
            float current = (analogVoltage * GAIN) / Rshunt;    // Ohm's law calculation
            return current;
        }
};
