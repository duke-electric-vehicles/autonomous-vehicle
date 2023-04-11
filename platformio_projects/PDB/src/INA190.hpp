#ifndef INA190_HPP
#define INA190_HPP

#include <Arduino.h>

/**
 * Code for the TI-INA190 current sense amplifier IC.
 **/
class INA190 {
    private:
        static const float SENSE_GAIN = 100.0;  // V/V
        static const float R_SHUNT = 0.001;     // Ohms
        int pin;

    public:
        INA190(int pin);

        float getCurrent();
};

#endif // INA190_HPP
