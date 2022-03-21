#include "Valve.h"
#include <Homie.h>

Valve::Valve()
: _node(NULL)
, _pin(0)
{}

Valve::Valve(const int* PIN, const char* ID, const char* name, HomieNode* node)
: _node(node)
, _pin(PIN)
, _ID(ID)
{

  pinMode(*_pin, OUTPUT);

  _node->advertise(ID)
          .setName(name)
          .setDatatype("boolean")
          .settable(  [PIN, this]( const HomieRange& range, const String& value) -> bool { 
            if (value != "true" && value != "false") return false;
            bool on = (value == "true");
            digitalWrite(*PIN, on ? HIGH : LOW);
            return true;
          });

}

bool Valve::open(bool open) {
  digitalWrite(*_pin, open ? HIGH : LOW);
  return digitalRead(*_pin);
}

bool Valve::publish() {
  _node->setProperty(_ID).send(String(digitalRead(*_pin)));
}