#include <DebugLog.h>

#include "eventloop.h"

static EventLoop eventLoop;

EventLoop& EventLoop::get() {
    return eventLoop;
}

void EventLoop::add(EventCallback callback, EventData data, EventPriority pri) {
    Event event;
    event.callback = callback;
    event.data = data;
    event.pri = pri;

    ensureNotFull();

    pri == EventPriority::PRIORITY_LOW ? lowPriEvents.push_back(event) : medPriEvents.push_back(event);
}

void EventLoop::loop() {
    if (!medPriEvents.empty()) {
        Event& event = medPriEvents.front();
        event.callback(event.data);
        medPriEvents.remove(0);
        return;
    }

    if (!lowPriEvents.empty()) {
        Event& event = lowPriEvents.front();
        event.callback(event.data);
        lowPriEvents.remove(0);
        return;
    }
}

void EventLoop::ensureNotFull() {
    if (!lowPriEvents.full() && !medPriEvents.full()) {
        return;
    }

    LOG_ERROR("Event loop full, clearing events");

    while (!lowPriEvents.empty() || !medPriEvents.empty()) {
        loop();
    }
}
