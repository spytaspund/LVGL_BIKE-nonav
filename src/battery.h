#ifndef Pangodream_18650_CL_h
#define Pangodream_18650_CL_h

#include "Arduino.h"

#define DEF_PIN 36
#define DEF_CONV_FACTOR 1.7
#define DEF_READS 20

/*
 * 18650 Ion-Li battery charge
 * Calculates charge level of an 18650 Ion-Li battery
 */
class Battery {    
  public:  
    
    /*
    * Constructor
    * @param addressPin, ADC pin number where the voltage divider is connected to
    */
    Battery(int addressPin);
    
    /*
    * Constructor
    * @param addressPin, ADC pin number where the voltage divider is connected to
    * @param convFactor, Convertion factor for analog read units to volts
    */
    Battery(int addressPin, double convFactor);
    
    /*
    * Constructor
    * @param addressPin, ADC pin number where the voltage divider is connected to
    * @param convFactor, Convertion factor for analog read units to volts
    * @param reads, Number of reads of analog pin to calculate an average value
    */
    Battery(int addressPin, double convFactor, int reads);
    /*
    * Constructor
    */
    Battery();    

    /*
     * Get the battery charge level (0-100)
     * @return The calculated battery charge level
     */
    int getBatteryChargeLevel();
    double getBatteryVolts();
    int getAnalogPin();
    int pinRead();
    double getConvFactor();
       
  private:

    int    _addressPin;               //!< ADC pin used, default is GPIO34 - ADC1_6
    int    _reads;                    //Number of reads of ADC pin to calculate an average value
    double _convFactor;               //!< Convertion factor to translate analog units to volts
    double _vs[101];                 //Array with voltage - charge definitions
    
    void   _initVoltsArray();
    int    _getChargeLevel(double volts);
    int    _analogRead(int pinNumber);
    double _analogReadToVolts(int readValue);
    
};

#endif