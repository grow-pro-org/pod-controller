#ifndef Valve_h
#define Valve_h

#include <Homie.h>

class Valve
{
  public:
    Valve();
    Valve(const int* PIN, const char* ID, const char* name, HomieNode* node);
    bool open(bool open);
    bool publish();
    
  private:
    HomieNode* _node;
    const int* _pin;
    const char* _ID;

};

#endif 