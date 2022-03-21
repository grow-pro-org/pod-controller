#include "Pump.h"
#include <Homie.h>

Pump::Pump()
: _node(NULL)
, _pin(0)
{}

Pump::Pump(const int* PIN, HomieNode* node)
: _node(node)
, _pin(PIN)
{

  pinMode(*_pin, OUTPUT);

  _node->advertise("run")
          .setName("Run pump")
          .setDatatype("boolean")
          .settable(  [PIN, this]( const HomieRange& range, const String& value) -> bool { 
            if (value != "true" && value != "false") return false;
            bool on = (value == "true");
            digitalWrite(*PIN, on ? HIGH : LOW);
            return true;
          });

}

bool Pump::run(bool run) {
  digitalWrite(*_pin, run ? HIGH : LOW);
  return digitalRead(*_pin);
}

bool Pump::publish() {
  _node->setProperty("run").send(String(digitalRead(*_pin)));
}