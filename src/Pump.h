#ifndef Pump_h
#define Pump_h

#include <Homie.h>

class Pump
{
  public:
    Pump();
    Pump(const int* PIN, HomieNode* node);
    bool run(bool open);
    bool publish();
    
  private:
    HomieNode* _node;
    const int* _pin;

};

#endif 