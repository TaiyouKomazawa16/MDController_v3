#ifndef MULTIFREQ_SMAG_MD_H_
#define MULTIFREQ_SMAG_MD_H_

#include <MotorDriver.h>
#include <PWM.h>
#include <Arduino.h>

//must not use 5 or 6 PWM pins in Arduino_UNO.

class MultiFreqSMagMD: public MotorDriver 
{
public:
    MultiFreqSMagMD(uint8_t, uint8_t, uint32_t);
        
protected:
    virtual void _drive(bool, uint8_t) override;
    virtual void _brake(uint8_t) override;
    virtual void _free() override;
        
private:
    uint8_t _dirPin;
    uint8_t _pwmPin;
};

#endif
