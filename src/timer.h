#include <inttypes.h>

class MetroTimer {
    public:
        MetroTimer(uint32_t interval_millis);
        bool check();
        void reset();
	
    private:
      bool firstCheck { true };
      uint32_t previousMillis { 0 };
      uint32_t intervalMillis { 0 };
};
