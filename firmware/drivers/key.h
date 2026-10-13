#ifndef KEY_H
#define KEY_H

typedef enum {
    KEY_NONE = 0,
    KEY_S1,
    KEY_S2,
    KEY_S3,
    KEY_S4,
} Key_Event;

/* Returns one debounced press event. S1 has the highest priority. */
Key_Event Key_GetEvent(void);

#endif
