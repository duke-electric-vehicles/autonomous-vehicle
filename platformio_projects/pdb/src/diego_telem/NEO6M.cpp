
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// note: may be optimal to call a neo6m.update() function in main loop, and 
// then store the output of the result in an instance variable. Then, whenever
// a getPos or getVel function is called, the GPS is not queried and instead
// reads the string based on the previously recorded value.

class NEO6M{
    private:
        static const int baudRate = 9600;

        float latitude;     // in degrees
        float longitude;    // in degrees
        float altitude;     // in feet
        float speed;        // in mph

        TinyGPSPlus gps;

    public:
        void initialize(){
            Serial2.begin(baudRate);
            Serial.println("GPS initialized");  // TODO: replace with dedicated log/debug library
        }

        float getLatitude(){
            return latitude;
        }

        float getLongitude(){
            return longitude;
        }

        float getAltitude(){
            return altitude;
        }

        float getSpeed(){
            return speed;
        }

        void update(){
            while (Serial2.available()) {
                Serial.print(Serial2.read());          // Print the received data to the USB serial monitor
            }

            // while(ss.available() > 0){      // read bitsream
            //     Serial.println("reading in GPS serial..");
            //     gps.encode(ss.read());

            //     if(gps.location.isUpdated()){  // bitstream has been decoded
            //         latitude = gps.location.lat();
            //         longitude = gps.location.lng();
            //         altitude = gps.altitude.feet();
            //         speed = gps.speed.mph();
                    
            //         Serial.println("GPS information updated");
            //         Serial.print("LAT="); Serial.print(latitude, 6);
            //         Serial.print("LNG="); Serial.println(longitude, 6);

            //         break;  // exit after recording values
            //     }
            //     // TODO: add error-catching for when GPS service/data may not be available.
            //}
        }

        void setNavUpdateRate(uint8_t rate) {
            uint16_t newRate_ms = 1000 / rate;
            uint8_t cfg_msg[] = {
                0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12
            };

            // Update the message with the new rate (in ms).
            cfg_msg[6] = newRate_ms & 0xFF;
            cfg_msg[7] = newRate_ms >> 8;

            // Update the message checksum.
            uint8_t a = 0, b = 0;
            for (int i = 2; i < 12; i++) {
                a += cfg_msg[i];
                b += a;
            }
            cfg_msg[12] = a;
            cfg_msg[13] = b;

            // Send the configuration message.
            for (int i = 0; i < sizeof(cfg_msg); i++) {
                Serial2.write(cfg_msg[i]);
            }
        }
};