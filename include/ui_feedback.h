#ifndef UI_FEEDBACK_H
#define UI_FEEDBACK_H

#include <Arduino.h>

class UIFeedback
{
public:
    // Initialize UI; pass button pin (optional, 0 = use default)
    static void init(uint8_t userButtonPin = 0);

    // Call from main loop to poll button; provide callbacks for short & long press
    static void loopPoll(void (*onShortPress)(), void (*onLongPress)());

    // Visual states
    static void showMessage(const char *msg);
    static void indicateListening();
    static void indicateAnalyzing();
    static void indicateAccessGranted();
    static void indicateAccessDenied();
    static void indicateEnrollStart();
    static void indicateEnrollDone();
    static void indicateEnrollFailed();
    static void resetSystem();
};

#endif // UI_FEEDBACK_H