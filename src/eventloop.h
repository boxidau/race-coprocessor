#include "Array.h"

typedef void* EventData;

typedef void (*EventCallback)(EventData);

// MEDIUM events pre-empt LOW. there is no HIGH because EventLoop should only be used for events
// which are non-critical.
enum EventPriority {
    PRIORITY_LOW = 0,
    PRIORITY_MEDIUM = 1,
};

struct Event {
    EventCallback callback;
    EventData data;
    EventPriority pri;
};

class EventLoop {
    public:
        static EventLoop& get();
        void add(EventCallback callback, EventData data, EventPriority pri);
        void loop();

    private:
        void ensureNotFull();

        Array<Event, 10> lowPriEvents;
        Array<Event, 10> medPriEvents;
};
