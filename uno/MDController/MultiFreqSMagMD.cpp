#include "MultiFreqSMagMD.h"

MultiFreqSMagMD::MultiFreqSMagMD(uint8_t dir, uint8_t pwm, uint32_t frequency)
: _dirPin(dir), _pwmPin(pwm)
{
    pinMode(_dirPin, OUTPUT);
    pinMode(_pwmPin, OUTPUT);
    InitTimersSafe();
    SetPinFrequencySafe(_pwmPin, frequency);
    
    _free();
}

void MultiFreqSMagMD::_drive(bool dir, uint8_t speed) 
{
    digitalWrite(_dirPin, dir ? LOW : HIGH);
    pwmWrite(_pwmPin, speed);
}

void MultiFreqSMagMD::_brake(uint8_t) 
{
    digitalWrite(_dirPin, HIGH);
    pwmWrite(_pwmPin, 0);
}

void MultiFreqSMagMD::_free() 
{
    digitalWrite(_dirPin, LOW);
    pwmWrite(_pwmPin, 0);
}
