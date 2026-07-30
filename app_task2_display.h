/*
 * app_task2_display.h - H task 2 OLED timing display
 */

#ifndef APP_TASK2_DISPLAY_H
#define APP_TASK2_DISPLAY_H

#include <stdbool.h>

/*
 * Initialize the SSD1306 and show the ready page.
 * Failure does not prevent the car task from running.
 */
bool Task2_DisplayInit(void);

/*
 * Refresh the display when the task state changes and update the running
 * time every 100 ms. Call once per main-loop iteration.
 */
void Task2_DisplayProcess(void);

bool Task2_DisplayIsOnline(void);

#endif /* APP_TASK2_DISPLAY_H */

