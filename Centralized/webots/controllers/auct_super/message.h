#include <stdint.h>
#include <stdarg.h>

// A message sent from the supervisor to the robot
typedef enum {
  MSG_EVENT_INVALID = 0, 
  MSG_EVENT_GPS_ONLY,
  MSG_EVENT_NEW,
  MSG_EVENT_WON,
  MSG_EVENT_DONE,
  MSG_QUIT,
  MSG_EVENT_BEING_HANDLED,
  MSG_BUNDLE_NEW,
  MSG_BUNDLE_WON,
  MSG_BUNDLE_DONE,
  MSG_BUNDLE_BEING_HANDLED
} message_event_state_t;

typedef enum {A = 0, B = 1} Event_type;

typedef struct {
  uint16_t robot_id; // id of the intended receiver
  double robot_x;
  double robot_y;
  double heading; // robot heading
  message_event_state_t event_state;
  // [[ valid if event_state > 0: 
  uint16_t event_id;
  double event_x;
  double event_y;
  Event_type event_type;

  // ]]
  int event_index;

  double a1_x;
  double a1_y;
  Event_type a1_type;
  double a2_x;
  double a2_y;
  Event_type a2_type;
  double a3_x;
  double a3_y;
  Event_type a3_type;

} message_t;

// A message sent from a robot to the supervisor
typedef struct {
  uint16_t robot_id; // id of the sender
  double value; // value of the bid
  int arrangement[3];
} bid_t;

void log_message(const char *format, ...) {
    FILE *file = fopen("debug_log.txt", "a");  // Open in append mode
    if (file) {
        va_list args;
        va_start(args, format);
        vfprintf(file, format, args);  // Write formatted output
        va_end(args);
        fprintf(file, "\n");  // Add a newline for better readability
        fclose(file);  // Ensure the file is properly closed
    } else {
        printf("Failed to open log file.\n");  // Optional error handling
    }
}