#include "Arduino.h"
#include "heltec.h"
#include <EEPROM.h>

#define VEXAS_LITE 1

#define LIST_SCREEN 0
#define TIMER_SCREEN 1
#define STOPWATCHES_SCREEN 2
#define SET_TIME_SCREEN 3
#define SET_NOTIFICATION_SCREEN 4
#define TIMER_SET_SCREEN 5
#define MENU_SCREEN 6
#define STRING_EDIT_SCREEN 7
#define TIME_DISPLAY_SCREEN 8
#define TODO_SCREEN 9
#define LIST_SEARCH_SCREEN 10
#define REFERENCE_LIST_SCREEN 11
#define REFERENCE_SEARCH_SCREEN 12
#define LONG_STRING_VIEW_SCREEN 13
#define LARGE_STRING_VIEW_SCREEN 14
#define LARGE_TIMER_VIEW_SCREEN 15
#define CALENDAR_VIEW_SCREEN 16
#define BLOB_SCREEN 17
#define DISPLAY_OFF_SCREEN 99
#define CLOSED 0
#define PENDING 1
#define OPEN 2
#define SNOOZED 3
#define INVALID_TIME 254

const uint8_t SCANNING_DELAY = 50;
const uint8_t MAX_ENTRIES = 120;
const uint8_t MSG_SIZE = 40;
const uint8_t REFERENCE_MSG_SIZE = 61;
const uint8_t MAX_TODO_ENTRIES = 10;
const uint8_t MAX_LIVE_EVENTS = 10;
const long BLOB_SIZE = 4000;
const uint8_t BLOB_EDIT_SIZE = 20;
const uint8_t BLOB_DISPLAY_SIZE = 125;
const uint8_t MAX_SCREEN_LINES= 7;
uint8_t MAX_SERIAL_LINES= 20;
const int SERIAL_DISPLAY_DELAY = 1000;
const uint8_t SERIAL_DISPLAY_CLS = 40;
// 1 second sleep has a dramatic effect on battery life
// uint8_t light_sleep_time_seconds = 1;
uint8_t light_sleep_time_seconds = 3;
const uint8_t light_sleep_time_seconds_default = light_sleep_time_seconds;
const uint8_t battery_warning_hours = 18;
uint8_t eeprom_written_this_battery_reset = 0;

uint8_t SERIAL_ESCAPE_SEQ = 0;

int DEFAULT_CPU_FREQ= 40;

const uint8_t MAX_DAYS_IN_THIS_MONTH[] = {1, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31, 1, 1, 1};

uint8_t time_month = 12;
uint8_t time_date = 14;
uint8_t time_weekday = 4;
uint8_t time_hours = 7;
uint8_t time_minutes = 45;
uint8_t time_seconds = 0;
uint8_t time_year = 23;
int key_repeat_delay = 800;
int key_typematic_delay = 50;
int incomingByte = 0; // for incoming serial data
long now = millis();
uint8_t eeprom_has_been_read = 0;

struct todo_entry
{
	uint8_t state;
	uint8_t date_closed;
	char message[MSG_SIZE];
};

struct button
{
	char letter;
	uint8_t usb_hid_hex;
	uint8_t ascii_hex;
};

struct appointment
{
	uint8_t hour;
	uint8_t minute;
	uint8_t month;
	uint8_t date;
	uint8_t weekday;
	uint8_t state;
	uint8_t snoozed_times;
	long snooze_time;
	uint8_t date_closed;
	uint8_t alarm_level;
	char message[MSG_SIZE];
};

#include "reference.h"

/*
	Dec  = Decimal Value
	Char = Character

	'5' has the int value 53
	if we write '5'-'0' it evaluates to 53-48, or the int 5
	if we write char c = 'B'+32; then c stores 'b'

	Dec  Char                           Dec  Char     Dec  Char     Dec  Char
	---------                           ---------     ---------     ----------
  0   NUL (null)                      32  SPACE     64  @         96  `
  1   SOH (start of heading)          33  !         65  A         97  a
  2   STX (start of text)             34  "         66  B         98  b
  3   ETX (end of text)               35  #         67  C         99  c
  4   EOT (end of transmission)       36  $         68  D        100  d
  5   ENQ (enquiry)                   37  %         69  E        101  e
  6   ACK (acknowledge)               38  &         70  F        102  f
  7   BEL (bell)                      39  '         71  G        103  g
  8   BS  (backspace)                 40  (         72  H        104  h
  9   TAB (horizontal tab)            41  )         73  I        105  i
	10  LF  (NL line feed, new line)    42  *         74  J        106  j
	11  VT  (vertical tab)              43  +         75  K        107  k
	12  FF  (NP form feed, new page)    44  ,         76  L        108  l
	13  CR  (carriage return)           45  -         77  M        109  m
	14  SO  (shift out)                 46  .         78  N        110  n
	15  SI  (shift in)                  47  /         79  O        111  o
	16  DLE (data link escape)          48  0         80  P        112  p
	17  DC1 (device control 1)          49  1         81  Q        113  q
	18  DC2 (device control 2)          50  2         82  R        114  r
	19  DC3 (device control 3)          51  3         83  S        115  s
	20  DC4 (device control 4)          52  4         84  T        116  t
	21  NAK (negative acknowledge)      53  5         85  U        117  u
	22  SYN (synchronous idle)          54  6         86  V        118  v
	23  ETB (end of trans. block)       55  7         87  W        119  w
	24  CAN (cancel)                    56  8         88  X        120  x
	25  EM  (end of medium)             57  9         89  Y        121  y
	26  SUB (substitute)                58  :         90  Z        122  z
	27  ESC (escape)                    59  ;         91  [        123  {
	28  FS  (file separator)            60  <         92  \        124  |
	29  GS  (group separator)           61  =         93  ]        125  }
	30  RS  (record separator)          62  >         94  ^        126  ~
	31  US  (unit separator)            63  ?         95  _        127  DEL
*/

const unsigned char watchlogo[] PROGMEM = { // NOTE THESE ARE MIRRORED!
	B00000000,
	B00111000,
	B01010100,
	B10010010,
	B10110010,
	B10000010,
	B01000100,
	B00111000
};

unsigned char batterylogo[] PROGMEM = { // NOTE THESE ARE MIRRORED!
	B00000000,
	B00111000,
	B01000100,
	B01000100,
	B01000100,
	B01000100,
	B01000100,
	B01111100
};

unsigned char timerlogochar[8] = {0};
uint8_t timer_logo_rotation = 0;

const unsigned char x_logo[] PROGMEM = { // NOTE THESE ARE MIRRORED!
	B00000000,
	B10000010,
	B01000100,
	B00101000,
	B00010000,
	B00101000,
	B01000100,
	B10000010
};

const unsigned char s_logo[] PROGMEM = { // NOTE THESE ARE MIRRORED!
	B00000000,
	B01111110,
	B11111111,
	B01110000,
	B00111100,
	B00001110,
	B11111111,
	B01111110
};

const unsigned char star_logo[] PROGMEM = { // NOTE THESE ARE MIRRORED!
	B00000000,
	B00010000,
	B00010000,
	B11111110,
	B00111000,
	B01111100,
	B11000110,
	B00000000
};

const unsigned char sadfaceinvert[] PROGMEM = { // NOTE THESE ARE MIRRORED!
	B00000000,   B00000000, 
	B11110000,   B00001111, 
	B00001100,   B00110000, 
	B00000010,   B01000000, 
	B01111001,   B10011110, 
	B01111101,   B10111110, 
	B00000001,   B10000000, 
	B11100001,   B10000111, 
	B00010001,   B10001000, 
	B00001001,   B10010000, 
	B11001001,   B10010011, 
	B11111001,   B10011111, 
	B00000010,   B01000000, 
	B00000100,   B00100000, 
	B11111000,   B00011111, 
	B00000000,   B00000000, 
};

const unsigned char sadface[] PROGMEM = { // NOTE THESE ARE MIRRORED!
	B11111111,   B11111111, 
	B00001111,   B11110000, 
	B11110011,   B11001111, 
	B11111101,   B10111111, 
	B10000110,   B01100001, 
	B10000010,   B01000001, 
	B11111110,   B01111111, 
	B00011110,   B01111000, 
	B11101110,   B01110111, 
	B11110110,   B01101111, 
	B00110110,   B01101100, 
	B00000110,   B01100000, 
	B11111101,   B10111111, 
	B11111011,   B11011111, 
	B00000111,   B11100000, 
	B11111111,   B11111111, 
};

const unsigned char happyface[] PROGMEM = { // NOTE THESE ARE MIRRORED!
	B11110000,  B00001111, 
	B00001100,  B00110000, 
	B00000010,  B01000000, 
	B00111001,  B10011100, 
	B01110101,  B10111010, 
	B01110101,  B10111010, 
	B00000001,  B10000000, 
	B00000001,  B10000000, 
	B11111111,  B10111111, 
	B00000101,  B10100000, 
	B10001001,  B10101111, 
	B01010001,  B10010000, 
	B11100010,  B01011111, 
	B00000100,  B00100000, 
	B11111000,  B00011111, 
	B00000000,  B00000000, 
};

unsigned char countdown_time_logo[] PROGMEM = { // NOTE THESE ARE MIRRORED!
	B00000000,
	B11111111,
	B10000010,
	B11111111,
	B00101000,
	B11111111,
	B00101000,
	B11111111
};
const struct button KEY_NULL = {0, 0x00, 0x00};

const struct button KEY_A = {'a', 0x04, 0x45};
const struct button KEY_B = {'b', 0x05, 0x45};
const struct button KEY_C = {'c', 0x06, 0x45};
const struct button KEY_D = {'d', 0x07, 0x45};
const struct button KEY_E = {'e', 0x08, 0x45};
const struct button KEY_F = {'f', 0x09, 0x45};
const struct button KEY_G = {'g', 0x0a, 0x45};
const struct button KEY_H = {'h', 0x0b, 0x45};
const struct button KEY_I = {'i', 0x0c, 0x45};
const struct button KEY_J = {'j', 0x0d, 0x45};
const struct button KEY_K = {'k', 0x0e, 0x45};
const struct button KEY_L = {'l', 0x0f, 0x45};
const struct button KEY_M = {'m', 0x10, 0x45};
const struct button KEY_N = {'n', 0x11, 0x45};
const struct button KEY_O = {'o', 0x12, 0x45};
const struct button KEY_P = {'p', 0x13, 0x45};
const struct button KEY_Q = {'q', 0x14, 0x45};
const struct button KEY_R = {'r', 0x15, 0x45};
const struct button KEY_S = {'s', 0x16, 0x45};
const struct button KEY_T = {'t', 0x17, 0x45};
const struct button KEY_U = {'u', 0x18, 0x45};
const struct button KEY_V = {'v', 0x19, 0x45};
const struct button KEY_W = {'w', 0x1a, 0x45};
const struct button KEY_X = {'x', 0x1b, 0x45};
const struct button KEY_Y = {'y', 0x1c, 0x45};
const struct button KEY_Z = {'z', 0x1d, 0x45};

const struct button KEY_1 = {'1', 0x2c, 0x45};
const struct button KEY_2 = {'2', 0x2c, 0x45};
const struct button KEY_3 = {'3', 0x2c, 0x45};
const struct button KEY_4 = {'4', 0x2c, 0x45};
const struct button KEY_5 = {'5', 0x2c, 0x45};
const struct button KEY_6 = {'6', 0x2c, 0x45};
const struct button KEY_7 = {'7', 0x2c, 0x45};
const struct button KEY_8 = {'8', 0x2c, 0x45};
const struct button KEY_9 = {'9', 0x2c, 0x45};
const struct button KEY_0 = {'0', 0x2c, 0x45};

const struct button KEY_TAB           = {'g', 0x2b, 0x00};
const struct button KEY_BRACKET_LEFT  = {'[', 0x00, 0x00};
const struct button KEY_BRACKET_RIGHT = {']', 0x00, 0x00};
const struct button KEY_ALT_LEFT      = {'g', 0x00, 0x00};
const struct button KEY_AT            = {'@', 0x00, 0x00};
const struct button KEY_PLUS          = {'+', 0x00, 0x00};
const struct button KEY_MINUS         = {'-', 0x00, 0x00};
const struct button KEY_SEMICOLON     = {';', 0x00, 0x00};
const struct button KEY_COLON         = {':', 0x00, 0x00};
const struct button KEY_APOSTROPHE    = {'g', 0x00, 0x00};
const struct button KEY_AMPERSAND     = {'&', 0x00, 0x00};
const struct button KEY_SHIFT_LEFT    = {'g', 0x00, 0x00};
const struct button KEY_COMMA         = {',', 0x00, 0x00};
const struct button KEY_PERIOD        = {'.', 0x00, 0x00};
const struct button KEY_SLASH         = {'/', 0x00, 0x00};
const struct button KEY_QUESTION      = {'?', 0x00, 0x00};
const struct button KEY_CONTROL_LEFT  = { 17, 0x00, 0x00};
const struct button KEY_GUI_LEFT      = {'g', 0x00, 0x00};
const struct button KEY_SPACE         = { 32, 0x2c, 0x00};
const struct button KEY_ALT_RIGHT     = {'g', 0x00, 0x00};
const struct button KEY_NONE          = {'g', 0x00, 0x00};
const struct button KEY_BACKSPACE     = {  8, 0x2a, 0x00};
const struct button KEY_DELETE        = {127, 0x2a, 0x00};
const struct button KEY_INSERT        = { 26, 0x2a, 0x00};
const struct button KEY_RETURN        = { 13, 0x28, 0x00};
const struct button KEY_ESCAPE        = { 27, 0x29, 0x00};

const struct button KEY_LEFT_ARROW    = { 18, 0x50, 0x00};
const struct button KEY_RIGHT_ARROW   = { 19, 0xef, 0x00};
const struct button KEY_DOWN_ARROW    = { 17, 0x51, 0x00};
const struct button KEY_UP_ARROW      = { 20, 0x52, 0x00};
const struct button KEY_BATTRST      = {177, 0x52, 0x00};

const uint8_t LAYERS = 2;
const uint8_t ROWS = 4; 
const uint8_t COLS = 10;

const char day_names[7][4] = {
	"SUN",
	"MON",
	"TUE",
	"WED",
	"THU",
	"FRI",
	"SAT",
};

// heltec esp32 v2
/* , , , 18, 19, , 22, 23 */
/* GPIOs 34-39 are input-only *and* do not have internal pull-up or pull-down circuitry.  */
/* const uint8_t colPins[COLS] = {17, 2, 5, 25,26,27,14,12,13,21}; */
// THIS IS for the prototype board!
/* const uint8_t colPins[COLS] = {17, 2, 5, 37,38,27,14,12,13,36}; */
/* USING PINS 1 AND 3 REQUIRES I DISABLE SERIAL MONITOR! */
/* pretty sure I also killed pin 26 with 5v...  */
/* const uint8_t colPins[COLS] = {17, 2, 5, 25,3,27,14,12,13,21}; // first board with shorts */
/* const uint8_t rowPins[ROWS] = {22, 19, 18, 23}; */
// THIS IS for the prototype board!
/* const uint8_t rowPins[ROWS] = {22, 21, 18, 23}; */
/* so I kind of think 19 and 18 are shorted on my esp32.  */
/* const uint8_t rowPins[ROWS] = {22, 1, 18, 23}; // first board with shorts */

// This is the version 1 with the wired shorts
// THIS REQUIRES serial to be disabled in the heltec begin function
#ifndef VEXAS_LITE
const uint8_t colPins[COLS] = {17, 2, 5, 25,3,27,14,12,13,21};
const uint8_t rowPins[ROWS] = {22, 1, 18, 23};
#endif

// THIS IS for the prototype board!
#ifdef VEXAS_LITE
// this is for vexas lite
const uint8_t colPins[COLS] = {17, 2, 5, 37,38,27,14,12,13,36};
const uint8_t rowPins[ROWS] = {22, 21, 18, 23};
// this is for interpres
// logical pin 26 (phys 24) needs to be swapped to Logical 15 (phys 
/* const uint8_t colPins[COLS] = {2,23,17,18,34,32,35,27,14,5}; */
/* const uint8_t rowPins[ROWS] = {22,19,33,13}; */
uint8_t ledPin = 25;
#endif

/* hey, 26 actually does work for pwm! */
uint8_t buzzerPin = 26;

// MOVED THIS TO REFERENCE.H
// THIS NOTE SYSTEM WASTES 41% OF THE FREE SPACE!
// REWRITE IT TO LIVE INDEX ON FOXES!
// char notes[MAX_ENTRIES][MSG_SIZE] = {
// "entry 1",
// "...",
// " ",
// " ",
// " ",
// "...",
// "entry 120",
// };

char timer_names[MAX_ENTRIES][MSG_SIZE] = {
	"cooking",
	"laundry",
	"3D print",
	"timer 3",
	"timer 4",
	"timer 5",
	"timer 6",
	"timer 7",
	"timer 8",
	"timer 9",
	"timer 10",
	"timer 11",
	"timer 12",
	"timer 13",
	"timer 14",
	"timer 15",
	"timer 16",
	"timer 17",
	"timer 18",
	"timer 19",
	"timer 20",
	"timer 21",
	"timer 22",
	"timer 23",
	"timer 24",
	"timer 25",
	"timer 26",
	"timer 27",
	"timer 28",
	"timer 29",
	"timer 30",
	"timer 31",
	"timer 32",
	"timer 33",
	"timer 34",
	"timer 35",
	"timer 36",
	"timer 37",
	"timer 38",
	"timer 39",
	"timer 40",
	"timer 41",
	"timer 42",
	"timer 43",
	"timer 44",
	"timer 45",
	"timer 46",
	"timer 47",
	"timer 48",
	"timer 49",
	"timer 50",
	"timer 51",
	"timer 52",
	"timer 53",
	"timer 54",
	"timer 55",
	"timer 56",
	"timer 57",
	"timer 58",
	"timer 59",
	"timer 60",
	"timer 61",
	"timer 62",
	"timer 63",
	"timer 64",
};

char watch_names[MAX_ENTRIES][MSG_SIZE] = {
	"cooking",
	"laundry",
	"3D print",
	"other",
	"drive home",
	"Last Restart",
	"battery",
	"watch 7",
	"watch 8",
	"watch 9",
	"watch 10",
	"watch 11",
	"watch 12",
	"watch 13",
	"watch 14",
	"watch 15",
	"watch 16",
	"watch 17",
	"watch 18",
	"watch 19",
	"watch 20",
	"watch 21",
	"watch 22",
	"watch 23",
	"watch 24",
	"watch 25",
	"watch 26",
	"watch 27",
	"watch 28",
	"watch 29",
	"watch 30",
	"watch 31",
	"watch 32",
	"watch 33",
	"watch 34",
	"watch 35",
	"watch 36",
	"watch 37",
	"watch 38",
	"watch 39",
	"watch 40",
	"watch 41",
	"watch 42",
	"watch 43",
	"watch 44",
	"watch 45",
	"watch 46",
	"watch 47",
	"watch 48",
	"watch 49",
	"watch 50",
	"watch 51",
	"watch 52",
	"watch 53",
	"watch 54",
	"watch 55",
	"watch 56",
	"watch 57",
	"watch 58",
	"watch 59",
	"watch 60",
	"watch 61",
	"watch 62",
	"watch 63",
	"watch 64",
	"watch 65",
	"watch 66",
	"watch 67",
	"watch 68",
	"watch 69",
	"watch 70",
	"watch 71",
	"watch 72",
	"watch 73",
	"watch 74",
	"watch 75",
	"watch 76",
	"watch 77",
	"watch 78",
	"watch 79",
	"watch 80",
	"watch 81",
	"watch 82",
	"watch 83",
	"watch 84",
	"watch 85",
	"watch 86",
	"watch 87",
	"watch 88",
	"watch 89",
	"watch 90",
	"watch 91",
	"watch 92",
	"watch 93",
	"watch 94",
	"watch 95",
	"watch 96",
	"watch 97",
	"watch 98",
	"watch 99",
};

/* char notescopy[MAX_ENTRIES][MSG_SIZE] = {0}; */
char entrycopyspace[MSG_SIZE] = {0};

char blob[BLOB_SIZE] = {' '};
char blob_edit[BLOB_EDIT_SIZE] = {0};
char blob_display[BLOB_DISPLAY_SIZE] = {0};

char menu_options[MAX_ENTRIES][MSG_SIZE] = {
	"Notes",
	"Timers",
	"Todos",
	"Stopwatches",
	"Calendar",
	"Sleep",
	"Reference",
	" ",
	" ",
	" ",
	"test eeprom write",
	"eeprom read",
	"eeprom Write Commit",
	"test eeprom commit",
	"invert display",
	"normal display",
	"buzzer chirp",
	"buzzer annoy",
	"buzzer notone",
	"buzzer permannoy",
	"test read",
	" ",
	"setBrightness(0)",
	"setBrightness(50)",
	"setBrightness(150)",
	"setBrightness(255)",
	"reset CPU 240",
	"reset CPU 80",
	"reset CPU 40",
	"reset CPU 10",
	" ",
	"-- hour ++",
	"-- minute ++ (timer)",
	"-- weekday ++",
	"-- day ++",
	" ",
	"reset",
	"serial note dump",
	"-- month ++",
	" ",
	"-- serial dispmax ++",
	"blob test",
	"blob dump",
	"blob reset",
	"check live events",
	"check const events",
	"reset seconds",
	" ",
	"eeprom read override",
	"menu 49",
	"menu 50",
	"menu 51",
	"menu 52",
	"menu 53",
	"menu 54",
	"menu 55",
	"menu 56",
	"menu 57",
	"menu 58",
	"menu 59",
	"menu 60",
	"menu 61",
	"menu 62",
	"menu 63",
	"menu 64",
	"menu 65",
	"menu 66",
	"menu 67",
	"menu 68",
	"menu 69",
	"menu 70",
	"menu 71",
	"menu 72",
	"menu 73",
	"menu 74",
	"menu 75",
	"menu 76",
	"menu 77",
	"menu 78",
	"menu 79",
	"menu 80",
	"menu 81",
	"menu 82",
	"menu 83",
	"menu 84",
	"menu 85",
	"menu 86",
	"menu 87",
	"menu 88",
	"menu 89",
	"menu 90",
	"menu 91",
	"menu 92",
	"menu 93",
	"menu 94",
	"menu 95",
	"menu 96",
	"menu 97",
	"menu 98",
	"menu 99",
	"menu 100",
	"menu 101",
	"menu 102",
	"menu 103",
	"menu 104",
	"menu 105",
	"menu 106",
	"menu 107",
	"menu 108",
	"menu 109",
	"menu 110",
	"menu 111",
	"menu 112",
	"menu 113",
	"menu 114",
	"menu 115",
	"menu 116",
	"menu 117",
	"menu 118",
	"menu 119",
};

uint8_t key_state[LAYERS][ROWS][COLS] = {{
		{0}, 
		{0}, 
		{0}, 
		{0}
	}, {
		{0}, 
		{0},
		{0}, 
		{0}
	}};

const uint8_t stuuf_size = 25;
uint8_t stuuf_position = 0;
uint8_t small_snooze_time = 1;
uint8_t snooze_time = 5;
uint8_t big_snooze_time = 60;
char stuuf[stuuf_size];

char (*string_entry_ptr)[MSG_SIZE];

char search_string[MSG_SIZE];

/* POINTER TIPS */
/* char (*ptr)[5] is a pointer to array, and char *ptr[5] is array of pointers. – Kos Sep 2 '11 at 7:53 */
/* When referencing entries within the array, dereference with the paren */
/* (*string_entry_ptr)[edit_position] = input; */
/* When referring to the array which heads the elements dereference WITHOUT the parens */
/* Heltec.display -> drawString(0, 0, String(*string_entry_ptr)); */
/* uint8_t edit_position = 0; */
uint8_t edit_position = 0;
long blob_position = 0;

uint8_t CURRENT_SCREEN = MENU_SCREEN;
uint8_t CURRENT_LAYER = 0;
long int CURRENT_SELECTION = 0;
long int EDIT_SELECTION = 0;
uint8_t LAST_SCREEN = 0; // assign this manually if you're going to skip screens
long timers [MAX_ENTRIES] = {0};
long timercopyspace = 0;
long alert_timeout = 0;
long watches [MAX_ENTRIES] = {0};
uint8_t PICK_TIME [4] = {0};
char cursor_swap_char = ' ';
char cursor_swap_char1 = ' ';
char LAST_INPUT = 0;
long LAST_INPUT_TIME = 0;
char LAST_SERIAL_INPUT = 0;
char LAST_SERIAL_INPUT1 = 0;
long LAST_SERIAL_INPUT_TIME = 0;
long LAST_SERIAL_DISPLAY_TIME = 0;
long last_minute_updated = 0;

long last_time_check = millis();
int brightness = 150;
long keypress = 0;

const struct event *sorted_calendar[MAX_EVENTS];



const struct button keys[LAYERS][ROWS][COLS] = {{
{KEY_Q, KEY_W, KEY_E,            KEY_R,      KEY_T,        KEY_Y,        KEY_U,         KEY_I,         KEY_O,      KEY_P         }, 
{KEY_A, KEY_S, KEY_D,            KEY_F,      KEY_G,        KEY_H,        KEY_J,         KEY_K,         KEY_L,      KEY_SEMICOLON }, 
{KEY_Z, KEY_X, KEY_C,            KEY_V,      KEY_B,        KEY_N,        KEY_M,         KEY_COMMA,     KEY_PERIOD, KEY_SLASH     }, 
{KEY_BRACKET_LEFT,
       KEY_BRACKET_RIGHT,
               KEY_CONTROL_LEFT, KEY_RETURN, KEY_SPACE,    KEY_SPACE,    KEY_BACKSPACE, KEY_RETURN    ,KEY_AMPERSAND,KEY_BACKSPACE }
	}, {                        // LT                                   // RT
{KEY_BATTRST,KEY_W, KEY_E,            KEY_R,      KEY_T,        KEY_Y,        KEY_7,         KEY_8,     KEY_9,      KEY_MINUS     }, 
{KEY_A,     KEY_S, KEY_D,            KEY_F,   KEY_LEFT_ARROW,KEY_RIGHT_ARROW,KEY_4,         KEY_5,      KEY_6,      KEY_PLUS      }, 
{KEY_Z,     KEY_X, KEY_C,            KEY_V,      KEY_B,        KEY_N,        KEY_1,         KEY_2,      KEY_3,      KEY_QUESTION  }, 
{KEY_2,KEY_3, KEY_CONTROL_LEFT,KEY_LEFT_ARROW,KEY_LEFT_ARROW,KEY_RIGHT_ARROW,KEY_RIGHT_ARROW,KEY_0,     KEY_PERIOD, KEY_DELETE    }
	}};                            // LT                            // RT

void setup(){
	int sorted_entry = 0;
	for(int month = 0; month < 13; month++){
		for(int date=0; date < 32; date++){
			// for X month and Y date
			for(long int calendar_entry=0; calendar_entry < MAX_EVENTS; calendar_entry++){
				// for all calendar entries
				if(events[calendar_entry].month == month && events[calendar_entry].date == date){
					// for this month and date, the calendar entry matches!
					// add pointer to the sorted_calendar list
					sorted_calendar[sorted_entry] = &events[calendar_entry];
					// move to next sorted calendar entry
					sorted_entry++;
				}
			}
		}
	}

	/* serial is disabled due to shorts on my esp32 board requiring me to use the tx and rx pins! */
	// enable serial for the prototype board!
#ifdef VEXAS_LITE
  Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Enable*/, true /*Serial Enable*/);
#endif
#ifndef VEXAS_LITE
  Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Enable*/, false /*Serial Enable*/);
#endif
	Serial.println("starting heltec libraries");
	watches[5] = millis();
	// ALL THE EEPROM STUFF IS TENTATIVE as the ESP32 does NOT have eeprom, and it is emulated
	// by the arduino libraries!
  /* EEPROM.begin(20); */
	/* WiFi.mode(WIFI_OFF); */
	/* btStop(); */
	/* setCpuFrequencyMhz(10); */
  /* if (!EEPROM.begin(MAX_ENTRIES * MSG_SIZE)) { */
	/* if (!EEPROM.begin(EEPROM_SIZE)) { */
  if (!EEPROM.begin(MAX_ENTRIES * MSG_SIZE)) {
		Heltec.display->drawString(0, 10, "eeprom begin FAIL!");
		delay(1000000);
	}
  else {
		Heltec.display->drawString(0, 10, "eeprom allocated!");
		delay(500);
	}
	Heltec.display->display();
  /* Heltec.display->flipScreenVertically(); */
  /* Heltec.display->setFont(ArialMT_Plain_10); */

	/* char testy[97] = {32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128};  */
	/* Heltec.display->drawStringMaxWidth(0, 0,DISPLAY_WIDTH,String(testy)); */
  /* Heltec.display->display(); */
  /* delay(500); */

  Serial.println("setting col pins");
  for(int i=0; i< COLS; i++)
		{
			Serial.println(colPins[i]);
			pinMode(colPins[i], INPUT_PULLUP);
		}
  Serial.println("setting row pins");
  for(int i=0; i< ROWS; i++)
		{
			Serial.println(rowPins[i]);
			pinMode(rowPins[i], OUTPUT);
			digitalWrite(rowPins[i], HIGH);
		}
  Serial.println("done setting pins");
	pinMode (buzzerPin, OUTPUT);

#ifdef VEXAS_LITE
	pinMode (ledPin, OUTPUT);
#endif

	// esp32 uses this led library for buzzing
  ledcSetup(0,1E5,12);
  ledcAttachPin(buzzerPin,0);

	for(int i=0; i< stuuf_size; i++)
		{
			stuuf[i] = 0;
		}
	timerlogochar[timer_logo_rotation] = timerlogochar[timer_logo_rotation] | 1;
	// blank out the speaker
	ledcWrite(0,0);
	/* CURRENT_SCREEN = DISPLAY_OFF_SCREEN;Heltec.display->displayOff(); */
	Heltec.display->setBrightness(brightness);
	/* setCpuFrequencyMhz(80); // this is a good balance */
	setCpuFrequencyMhz(DEFAULT_CPU_FREQ); // but lets see how unbalanced this really is...
	permannoy();
  Serial.println(*(sorted_calendar[0])->message);
}

void loop()
{
	// calculate time delta to increment clock
	now = millis();
	int delta_millis = now - last_time_check;
	/* Serial.println(delta_millis); */
	// 3358
	if (delta_millis > 999){
		// more than a second has passed
		int seconds_already_counted = 1000;
		time_seconds++;
		/* if (delta_millis/1000) */
		if (delta_millis > 1999){time_seconds++;seconds_already_counted = seconds_already_counted + 1000;}
		if (delta_millis > 2999){time_seconds++;seconds_already_counted = seconds_already_counted + 1000;}
		if (delta_millis > 3999){time_seconds++;seconds_already_counted = seconds_already_counted + 1000;}
		if (delta_millis > 4999){time_seconds++;seconds_already_counted = seconds_already_counted + 1000;}
		if (delta_millis > 5999){time_seconds++;seconds_already_counted = seconds_already_counted + 1000;}
		// this code will check the delta for additional fractions
		// of a second and add them to the past time, so we can count
		// fractions into the future
		last_time_check = now - (delta_millis - seconds_already_counted);
		if(time_seconds > 59){time_seconds=time_seconds-60;time_minutes++;
			if(time_minutes > 59){time_minutes=time_minutes-60;time_hours++;
				if(time_hours > 23){ // one day ticks over to the next
					time_hours=0;
					time_weekday++;
					if(time_weekday > 6){
						time_weekday=0;
					}
					time_date++;
					if(time_date > MAX_DAYS_IN_THIS_MONTH[time_month]){
						time_date=1;
						time_month++;
					}
					/* Serial.println("resetting todos"); */
					for(int x=0; x < MAX_TODO_ENTRIES; x++){
						// reset todos daily
						todos[x].state = OPEN;
					}
					/* Serial.print("checking events"); */
					check_events();
					// check_these_events();
				}
			}
		}
	}
	/* else if(now < last_time_check){ */
	/* do long rollover thingy (50 days???) */
	/* - [X] always record previous loop milli */
	/* - [X] always compare previous to current, if current smaller: */
	/*   - [ ] subtract max from previous milli and add to current milli */
	/*   - [ ] this gives delta of time elapsed THEN add to day ticket */
	/* - set the time and traverse LONG expiration */
	/* } */
  /* Serial.print("."); */
  // GPIO is active low     
  // if (Serial.available() > 0) {
  //   // read the incoming byte:
  //   incomingByte = Serial.read();
  //   // say what you got:
  //   Serial.print("I received: ");
  //   Serial.println(incomingByte, DEC);
  // }
  for(int row=0; row< ROWS; row++) {
		digitalWrite(rowPins[row], LOW);
		for(int col=0; col< COLS; col++)
			{
				if ((digitalRead(colPins[col]) == LOW) && (key_state[CURRENT_LAYER][row][col]== 0)){
					// key was just pressed, set the state
					key_state[CURRENT_LAYER][row][col]= 1;
					char input = keys[CURRENT_LAYER][row][col].letter;
					// set simple repeat 
					if (keypress > now){  // special state of last keypress with future time so go typematic
						// repeat will happen when key held longer than repeat delay
						//         now  - ( 800            -    100)
						//         now  - (700) this will trigger repeat delay 100ms EARLIER
						keypress = now - (key_repeat_delay - key_typematic_delay);
					}
					else{ // standard reset the time
						keypress = now; // must hold for a full key_repeat_delay to go typematic
					}
					if (input == 17){CURRENT_LAYER = 1;}
					process_input(input);
				}
				else if ((digitalRead(colPins[col]) == HIGH) && (key_state[CURRENT_LAYER][row][col]== 1)){
					// key was pressed and just released
					key_state[CURRENT_LAYER][row][col] = 0;
					if (keys[CURRENT_LAYER][row][col].letter != 17){
						// key was pressed and just released, and it's not a special layer key
						LAST_INPUT = keys[CURRENT_LAYER][row][col].letter;
						LAST_SERIAL_INPUT = 0; // wipe out last serial input when typing on hard keyboard
					}
					else{ // if keystroke is 17 (or other layer)
						CURRENT_LAYER = 0;
					}
				}
				else if (digitalRead(colPins[col]) == LOW // button is pressed
								 && key_state[CURRENT_LAYER][row][col]== 1 // button already pressed
								 && now - keypress > key_repeat_delay  // press is over repeat delay time
								 && keys[CURRENT_LAYER][row][col].letter != 17 // don't do typamatic if modifier
								 ){ // and button is NOT the modifier button
					key_state[CURRENT_LAYER][row][col] = 0; // reset state so it processes keypress again
					keypress = now + 500; // set keypress to future to trigger logic
				}
			}
		digitalWrite(rowPins[row], HIGH);
	}
	while (Serial.available()) {
	 	char input = Serial.read();
		LAST_SERIAL_INPUT_TIME = now;
		// special input processing \r for newline, \b for backspace
	 	if (input != 92 && input != 10){ // if receiving backslash or newline via serial, do not process as input
	 				if (LAST_SERIAL_INPUT == 92 && input == 'r'){input = KEY_RETURN.letter;process_input(input);}
	 				else if (LAST_SERIAL_INPUT == 92 && input == 'b'){input = KEY_BACKSPACE.letter;process_input(input);}
					else{ // not processing special input
					   if(input == 27){
				        	SERIAL_ESCAPE_SEQ++; // rec'd escape sequence, increment flag
									input = 0;	// dump the input
				     }
					   else if(SERIAL_ESCAPE_SEQ == 1 && input == 91){
					   			SERIAL_ESCAPE_SEQ++; // recd escape seq THEN [, increment flag
									input = 0;	// dump the input
        	   }
					   else if(SERIAL_ESCAPE_SEQ == 2 && input == 68){
					   			SERIAL_ESCAPE_SEQ = 0;
									input = 18; // it's a left arrow!
        	   }
					   else if(SERIAL_ESCAPE_SEQ == 2 && input == 67){
					   			SERIAL_ESCAPE_SEQ = 0;
					        input = 19; // it's a right arrow!
        	   }
					   else if(SERIAL_ESCAPE_SEQ == 2 && input == 66){
					   			SERIAL_ESCAPE_SEQ = 0;
									input = 17; // it's a left arrow?
        	   }
					   else if(SERIAL_ESCAPE_SEQ == 2 && input == 65){
					   			SERIAL_ESCAPE_SEQ = 0;
					        input = 20; // it's a right arrow?
        	   }
					   else{
         					SERIAL_ESCAPE_SEQ = 0; // NOT an escape sequence, reset it
				     }
	     	     process_input(input);
	 			}
	 			// I guess we have to track the last inputs for chained commands like dd
	 			// but still ignore backslash and newline for any chained commands
	 			LAST_INPUT = input; 
	 	}
	 	LAST_SERIAL_INPUT1 = LAST_SERIAL_INPUT;
	 	LAST_SERIAL_INPUT = input;
		/* if (input == 13) {Serial.print("/");} */
		/* else{Serial.print(".");} */
		// delay(5); // slow down serial read just a bit // nah, doesn't need it!
	}
	if (CURRENT_SCREEN != DISPLAY_OFF_SCREEN){
		draw_screen(CURRENT_SCREEN);
		delay(SCANNING_DELAY); // slow it down just for the sake of battery life
	}
	else{
		// asleep, so check for expired timers
		uint8_t wake = 0;
		for (int e = 0; e < MAX_ENTRIES-1; e++){
			digitalWrite(ledPin, HIGH);
			if (timers[e] < now && timers[e] != 0){ // time is OVER
				wake = 1;
				break;
				}
			}
		// led pin is set low on sleep automatically
		digitalWrite(ledPin, LOW);
		// or does it???
		if ((((now - watches[6])/1000)/3600) >= battery_warning_hours){ // calculation to get hours from stopwatch
			if (light_sleep_time_seconds == light_sleep_time_seconds_default){
				light_sleep_time_seconds = light_sleep_time_seconds * 5;
				setCpuFrequencyMhz(10);
			}
			if (!eeprom_written_this_battery_reset){
				// if we enter low power mode, write eeprom
				eeprom_write_commit();
				eeprom_written_this_battery_reset = 1;
			}
			if (now - alert_timeout > (10 * 1000)){ // make a noise every 10 seconds while sleeping
				annoy();
				alert_timeout =  now;
			}
			// battery stopwatch over time while sleeping, do sleeping warning
			digitalWrite(ledPin, HIGH);
		}
		if (wake){ // time is OVER
			light_sleep_time_seconds = light_sleep_time_seconds_default;
			/* reset_selection_and_change_screen(TIME_DISPLAY_SCREEN, 0); */
			/* Heltec.display->displayOn(); */
			setCpuFrequencyMhz(DEFAULT_CPU_FREQ);
			exit_light_sleep();
			draw_countdown_time(); // technically means blinky lights will be delayed when processed
		}
		else{ // no timers to wake up for
			/* Serial.println("sleeping"); */
			/* delay(2000); // don't even process the display if it's off */
			esp_sleep_enable_timer_wakeup(light_sleep_time_seconds * 1000000); // ESP32 wakes up every 2 seconds
			esp_light_sleep_start();
		}
	}
}

void draw_screen(uint8_t screen) {
  Heltec.display->clear();
	if (CURRENT_SCREEN == MENU_SCREEN){ //                              DRAW MENU SCREEN
		Heltec.display->setFont(ArialMT_Plain_10);
		oled_print_char_array_range(menu_options, CURRENT_SELECTION);
	}
	else if (CURRENT_SCREEN == LIST_SCREEN){ //                         DRAW LIST SCREEN
		Heltec.display->setFont(ArialMT_Plain_10);
		oled_print_char_array_range(notes, CURRENT_SELECTION);
	}
	else if (CURRENT_SCREEN == REFERENCE_LIST_SCREEN){ //               DRAW REFERENCE LIST  SCREEN
		Heltec.display->setFont(ArialMT_Plain_10);
		oled_print_reference(REFERENCE, CURRENT_SELECTION);
	}
	else if (CURRENT_SCREEN == CALENDAR_VIEW_SCREEN){ //               DRAW CALENDAR LIST  SCREEN
		Heltec.display->setFont(ArialMT_Plain_10);
		oled_print_calendar(CURRENT_SELECTION);
	}
	else if (CURRENT_SCREEN == TODO_SCREEN){ //                        DRAW TODOS SCREEN
		Heltec.display->setFont(ArialMT_Plain_10);
		int entry_offset_counter = 0;
		long int start = get_start_range_for_selection(MAX_TODO_ENTRIES, CURRENT_SELECTION, MAX_SCREEN_LINES);
		for(int entry=start; entry < (start+MAX_SCREEN_LINES); entry++){
			if (entry >= 0 && entry < MAX_TODO_ENTRIES){ // actually a potential for fewer todos than display size
				if (entry == CURRENT_SELECTION){ // this is the current selection, so invert it! 
					Heltec.display->setColor(WHITE);
					Heltec.display->fillRect(0, (entry_offset_counter*8)+3, DISPLAY_WIDTH, 8);
					Heltec.display->setColor(BLACK);
				}
				Heltec.display -> drawString(0, entry_offset_counter*8, String(
																																			 ((todos[entry].state == OPEN) ? "[   ] " : "[X] ")
																																			 + String(todos[entry].message)
																																			 )); // is this better notation?
				if (entry == CURRENT_SELECTION){ // done with selection so UN invert
					Heltec.display->setColor(WHITE);
				}
			}
			entry_offset_counter++;
		}
		if (LAST_SERIAL_INPUT != 0 && LAST_SERIAL_INPUT_TIME > LAST_SERIAL_DISPLAY_TIME){
			// draw_serial_output
			serial_cls();
			long int start = get_start_range_for_selection(MAX_TODO_ENTRIES, CURRENT_SELECTION, MAX_SERIAL_LINES);
			for(int entry=start; entry < (start+MAX_SERIAL_LINES); entry++){
				char first_character = 32;
				if (entry == CURRENT_SELECTION){
					first_character = '>'; // preceed line with > character
				}
				/* Serial.print(F(notes[i])); */
				if (entry >= 0 && entry < MAX_TODO_ENTRIES){ // actually a potential for fewer todos than display size
					Serial.print(first_character);
					Serial.print((todos[entry].state == OPEN) ? "[ ] " : "[X] ");
					Serial.println(todos[entry].message);
				}
				entry_offset_counter++;
			}
			LAST_SERIAL_DISPLAY_TIME = now;
		}
	}
	else if (CURRENT_SCREEN == TIMER_SCREEN){ //                        DRAW TIMER SCREEN
		Heltec.display->setFont(ArialMT_Plain_10);
		oled_print_long_array_range(timers, CURRENT_SELECTION);
	}
	else if (CURRENT_SCREEN == LARGE_TIMER_VIEW_SCREEN){ //                        DRAW LARGE TIMER VIEW SCREEN
		display_timer_full_screen(timers, CURRENT_SELECTION);
	}
	else if (CURRENT_SCREEN == STOPWATCHES_SCREEN){ //                  DRAW STOP WATCHES SCREEN
		Heltec.display->setFont(ArialMT_Plain_10);
		oled_print_long_array_range_stopwatches(watches, CURRENT_SELECTION);
	}
	else if (CURRENT_SCREEN == LIST_SEARCH_SCREEN){ //                       DRAW LIST SEARCH SCREEN
		Heltec.display->setFont(ArialMT_Plain_10);
		int search_string_length = 0;
		long int entries_matched = 0;
		for (int letter = 0; letter < MSG_SIZE; letter++)
			if(search_string[letter] != 0){
				search_string_length++;
			}
		int entry_offset_counter = 0;
		Heltec.display -> drawString(0, 0, String(search_string));
		entry_offset_counter++;
		int search_string_field_position = 0;
		for (int entry = 0; entry < MAX_ENTRIES; entry++){
			// for each entry
			for (int letter = 0; letter < (MSG_SIZE - search_string_length); letter++){
				// for each letter in each entry
				// if the letter matches the first entry in search string, and search string is NOT blank
				if(notes[entry][letter] == search_string[0] && search_string[0] != 0){
					// got a match!
					// search forward the search string length
					int match_char_count = 1; // one char already matches
					for (int match_position = 1; match_position < search_string_length; match_position++){
						if(notes[entry][letter+match_position] == search_string[match_position]){
							match_char_count++; // one more successful match
						}
						if (match_char_count == search_string_length){
							// entry match success!
							entries_matched++; // RELATIVE count of matches to match current selection highlighting
							if (entries_matched > (CURRENT_SELECTION - (MAX_SCREEN_LINES/2))){
								// don't even draw the entry unless it's near the current selection
								if (entries_matched == CURRENT_SELECTION){ // this is the current selection, so invert it! 
									Heltec.display->setColor(WHITE);
									Heltec.display->fillRect(0, (entry_offset_counter*8)+3, DISPLAY_WIDTH, 8);
									Heltec.display->setColor(BLACK);
									EDIT_SELECTION = entry; // for current selection set edit selection to ACTUAL reference entry number
								}
								Heltec.display -> drawString(0, entry_offset_counter*8, notes[entry]); // is this better notation?
								if (entries_matched == CURRENT_SELECTION){ // done with selection so UN invert
									Heltec.display->setColor(WHITE);
								}
								entry_offset_counter++; // successful print, increment print offset
							}
						}
					}
				}
			}
			// done searching this entry, reset search field
			search_string_field_position = 0;
			if(entry_offset_counter > MAX_SCREEN_LINES) {break;}
		}
	}
	else if (CURRENT_SCREEN == REFERENCE_SEARCH_SCREEN){ //                       DRAW REFERENCE SEARCH SCREEN
		Heltec.display->setFont(ArialMT_Plain_10);
		int search_string_length = 0;
		long int entries_matched = 0;
		for (int letter = 0; letter < MSG_SIZE; letter++)
			if(search_string[letter] != 0){
				search_string_length++;
			}
		int entry_offset_counter = 0;
		Heltec.display -> drawString(0, 0, String(search_string));
		entry_offset_counter++;
		int search_string_field_position = 0;
		for (int entry = 0; entry < MAX_REFERENCE_ENTRIES; entry++){
			// for each entry
			for (int letter = 0; letter < (MSG_SIZE - search_string_length); letter++){
				// for each letter in each entry
				// if the letter matches the first entry in search string, and search string is NOT blank
				if(REFERENCE[entry][letter] == search_string[0] && search_string[0] != 0){
					// got a match!
					// search forward the search string length
					int match_char_count = 1; // one char already matches
					for (int match_position = 1; match_position < search_string_length; match_position++){
						if(REFERENCE[entry][letter+match_position] == search_string[match_position]){
							match_char_count++; // one more successful match
						}
						if (match_char_count == search_string_length){
							// entry match success!
							entries_matched++; // RELATIVE count of matches to match current selection highlighting
							if (entries_matched > (CURRENT_SELECTION - (MAX_SCREEN_LINES/2))){
								// don't even draw the entry unless it's near the current selection
								if (entries_matched == CURRENT_SELECTION){ // this is the current selection, so invert it! 
									Heltec.display->setColor(WHITE);
									Heltec.display->fillRect(0, (entry_offset_counter*8)+3, DISPLAY_WIDTH, 8);
									Heltec.display->setColor(BLACK);
									EDIT_SELECTION = entry; // for current selection set edit selection to ACTUAL reference entry number
								}
								Heltec.display -> drawString(0, entry_offset_counter*8, REFERENCE[entry]); // is this better notation?
								if (entries_matched == CURRENT_SELECTION){ // done with selection so UN invert
									Heltec.display->setColor(WHITE);
								}
								entry_offset_counter++; // successful print, increment print offset
							}
						}
					}
				}
			}
			// done searching this entry, reset search field
			search_string_field_position = 0;
			if(entry_offset_counter > MAX_SCREEN_LINES) {break;}
		}
	}
	else if (CURRENT_SCREEN == TIMER_SET_SCREEN){ //                    DRAW TIMER SET SCREEN
		Heltec.display->setFont(ArialMT_Plain_24);
    // Heltec.display -> drawString(0, 0,
		// 														 ((PICK_TIME[0] < 10) ? '0' + String(PICK_TIME[0]) : String(PICK_TIME[0]))
		// 														 + ':'
		// 														 + ((PICK_TIME[1] < 10) ? '0' + String(PICK_TIME[1]) : String(PICK_TIME[1]))
		// 														 );
		Heltec.display->drawRect(CURRENT_SELECTION*32, 0, 26, 30);
		uint8_t printx = 0;
		uint8_t printy = DISPLAY_HEIGHT/2;
		for(int field=0; field < 4; field++){
			if(field == 2){
				Heltec.display -> drawString(printx, printy,":");
				printx = printx + 13;
			}
			if(CURRENT_SELECTION == field){
				Heltec.display->fillRect(printx, printy, 13, 30);
				Heltec.display->setColor(BLACK);
				Heltec.display->drawString(printx, printy,String(PICK_TIME[field]));
				Heltec.display->setColor(WHITE);
			}
			else{
				Heltec.display->drawString(printx, printy,String(PICK_TIME[field]));
			}
			printx = printx + 13;
		}
		if (LAST_SERIAL_INPUT != 0 && LAST_SERIAL_INPUT_TIME > LAST_SERIAL_DISPLAY_TIME){
			// draw_serial_output single line
			serial_cls();
			for(int field=0; field < 4; field++){
				if(field == 2){
					Serial.print(':');
					Serial.print(PICK_TIME[field]);
				}
				else{
					Serial.print(PICK_TIME[field]);
				}
			}
			Serial.println(' ');
			for(int x=0; x < CURRENT_SELECTION; x++){
				Serial.print(' ');
			}
			if(CURRENT_SELECTION >= 2){
				Serial.print(' ');
			}
			Serial.println('^');
			LAST_SERIAL_DISPLAY_TIME = now;
		}
	}
	else if (CURRENT_SCREEN == LONG_STRING_VIEW_SCREEN){ //                  DRAW STRING EDIT SCREEN
		/* Heltec.display->setFont(ArialMT_Plain_10); */
		Heltec.display->setFont(ArialMT_Plain_16);
		Heltec.display -> drawStringMaxWidth(0, 0, DISPLAY_WIDTH, REFERENCE[CURRENT_SELECTION]);
	}
	else if (CURRENT_SCREEN == LARGE_STRING_VIEW_SCREEN){ //                  DRAW LARge NOTE STRING view SCREEN
		/* Heltec.display->setFont(ArialMT_Plain_10); */
		Heltec.display->setFont(ArialMT_Plain_16);
		/* Heltec.display->setFont(ArialMT_Plain_24); */
		Heltec.display -> drawStringMaxWidth(0, 0, DISPLAY_WIDTH, notes[CURRENT_SELECTION]);
	}
	else if (CURRENT_SCREEN == LARGE_STRING_VIEW_SCREEN){ //                  DRAW STRING EDIT SCREEN
		Heltec.display->setFont(ArialMT_Plain_10);
		Heltec.display -> drawStringMaxWidth(0, 0, DISPLAY_WIDTH, REFERENCE[CURRENT_SELECTION]);
	}
	else if (CURRENT_SCREEN == STRING_EDIT_SCREEN){ //                  DRAW STRING EDIT SCREEN
		Heltec.display->setFont(ArialMT_Plain_10);
		/* Heltec.display->setColor(WHITE); */
		/* Heltec.display -> drawString(0, 0, String(*string_entry_ptr)); */
		Heltec.display -> drawStringMaxWidth(0, 0, DISPLAY_WIDTH, String(*string_entry_ptr));
		/* Heltec.display -> drawString(0, 0, notes[EDIT_SELECTION]); */
		/* Heltec.display -> drawStringMaxWidth(0, 0, DISPLAY_WIDTH, String((*string_entry_ptr)[EDIT_SELECTION])); // is this better notation? */
		/* Heltec.display->fillRect(edit_position, 9, 8, 2); */
		int cursor_width = Heltec.display->getStringWidth(*string_entry_ptr+(edit_position*sizeof(char)), 1);
		if(cursor_width == 0){
			cursor_width = 5;
		}
		Heltec.display->fillRect(
														 Heltec.display->getStringWidth(*string_entry_ptr, edit_position),
														 11,
														 cursor_width,
														 2);
		/* Heltec.display->setColor(BLACK); */
		Heltec.display->drawString(0, 30, String(edit_position));
		Heltec.display->drawString(20, 30, String(cursor_width));
		/* Heltec.display->setColor(WHITE); */
		if (LAST_SERIAL_INPUT != 0 && LAST_SERIAL_INPUT_TIME > LAST_SERIAL_DISPLAY_TIME){
			// draw_serial_output single line
			serial_cls();
			Serial.println(*string_entry_ptr);
			// print cursor (edit) position
			for(int x=0; x < edit_position; x++){
				Serial.print(' ');
			}
			Serial.print('^');
			LAST_SERIAL_DISPLAY_TIME = now;
		}
	}
	else if (CURRENT_SCREEN == BLOB_SCREEN){ //                  DRAW BLOB SCREEN
		Heltec.display->setFont(ArialMT_Plain_10);
		// long start_char = get_start_range_for_selection(BLOB_SIZE, blob_position, BLOB_EDIT_SIZE);
		// for (int m = 0; m < BLOB_EDIT_SIZE; m++){
		// 	blob_edit[m] = blob[start_char];
		// 	start_char++;
		// }
		// Heltec.display -> drawStringMaxWidth(0, 0, DISPLAY_WIDTH, String(blob_edit));

		long start_char = get_start_range_for_selection(BLOB_SIZE, blob_position, BLOB_DISPLAY_SIZE);
		long i = start_char;
		for (int m = 0; m < BLOB_DISPLAY_SIZE; m++){
			blob_display[m] = blob[i];
			i++;
		}
		Heltec.display -> drawStringMaxWidth(0, 0, DISPLAY_WIDTH, String(blob_display));
		cursor_swap_char = blob_display[blob_position-start_char];
		cursor_swap_char1 = blob_display[blob_position+1-start_char];
		blob_display[blob_position-start_char] = '_';
		blob_display[blob_position+1-start_char] = 0;
		Heltec.display -> drawStringMaxWidth(0, 0, DISPLAY_WIDTH, String(blob_display));
		blob_display[blob_position-start_char] = cursor_swap_char;
		blob_display[blob_position+1-start_char] = cursor_swap_char1;
		/* Heltec.display -> drawString(0, 0, notes[EDIT_SELECTION]); */
		/* Heltec.display -> drawStringMaxWidth(0, 0, DISPLAY_WIDTH, String((*string_entry_ptr)[EDIT_SELECTION])); // is this better notation? */
		/* Heltec.display->fillRect(blob_position, 9, 8, 2); */
		// int cursor_width = Heltec.display->getStringWidth(blob+(blob_position*sizeof(char)), 1);
		// if(cursor_width == 0){cursor_width = 5;}
		//Heltec.display->fillRect(
		//												 Heltec.display->getStringWidth(blob_edit, BLOB_SIZE - blob_position),
		//												 11,
		//												 cursor_width,
		//												 2);
		/* Heltec.display->setColor(BLACK); */
		Heltec.display->drawString(0, DISPLAY_HEIGHT-11, String(BLOB_SIZE - blob_position));
		Heltec.display->drawString(DISPLAY_WIDTH/2, DISPLAY_HEIGHT-11, String(start_char));
		/* Heltec.display->setColor(WHITE); */
		if (LAST_SERIAL_INPUT != 0 && LAST_SERIAL_INPUT_TIME > LAST_SERIAL_DISPLAY_TIME){
			// draw_serial_output single line
			serial_cls();
			Serial.println(blob);
			// print cursor (edit) position
			for(long x=0; x < blob_position; x++){
				Serial.print(' ');
			}
			Serial.print('^');
			LAST_SERIAL_DISPLAY_TIME = now;
		}
	}
	else if (CURRENT_SCREEN == TIME_DISPLAY_SCREEN){ //                  DRAW TIME DISPLAY SCREEN
		int radius = 30;
		int centerX = DISPLAY_WIDTH-radius-1; // right align clock face with 1 more for rounding
		int centerY = DISPLAY_HEIGHT/2;
		int analog_hours;
		if(time_hours == 0){analog_hours = 12;}
		else if(time_hours > 12){analog_hours = time_hours - 12;}
		else analog_hours = time_hours;
		Heltec.display->drawCircle(centerX, centerY, radius);
		Heltec.display->drawCircle(centerX, centerY, 2);
		//hour ticks
		for( int z=0; z < 360;z= z + 30 ){
			//Begin at 0° and stop at 360°
			float angle = z ;
			angle=(angle/57.29577951) ; //Convert degrees to radians
			int x2=(centerX+(sin(angle)*radius));
			int y2=(centerY-(cos(angle)*radius));
			int x3=(centerX+(sin(angle)*(radius-5)));
			int y3=(centerY-(cos(angle)*(radius-5)));
			Heltec.display->drawLine(x2,y2,x3,y3);
		}
		// display second hand
		float angle = time_seconds*6 ;
		angle=(angle/57.29577951) ; //Convert degrees to radians  
		int x3=(centerX+(sin(angle)*(radius)));
		int y3=(centerY-(cos(angle)*(radius)));
		Heltec.display->drawLine(centerX,centerY,x3,y3);
		// display minute hand
		angle = time_minutes * 6 ;
		angle=(angle/57.29577951) ; //Convert degrees to radians  
		x3=(centerX+(sin(angle)*(radius-3)));
		y3=(centerY-(cos(angle)*(radius-3)));
		Heltec.display->drawLine(centerX,centerY,x3,y3);
		// display hour hand
		angle = time_hours * 30 + int((time_minutes / 12) * 6 )   ;
		angle=(angle/57.29577951) ; //Convert degrees to radians  
		x3=(centerX+(sin(angle)*(radius-11)));
		y3=(centerY-(cos(angle)*(radius-11)));
		Heltec.display->drawLine(centerX,centerY,x3,y3);
		// print time at top of screen
		Heltec.display->setFont(ArialMT_Plain_24);
		Heltec.display -> drawString(0, 0,
																 String(
																				// String(time_hours) // add a zero instead for military time
																				((time_hours < 10) ? '0' + String(time_hours) : String(time_hours))
																				+ ":"
																				+ ((time_minutes < 10) ? '0' + String(time_minutes) : String(time_minutes))
																				// + ":" // remove seconds because we draw seconds on analog clock
																				/* + ((time_seconds < 10) ? '0' + String(time_seconds) : String(time_seconds)) */
																				)
																 );
		// print the numerical date in the middle of the screen
		Heltec.display->setFont(ArialMT_Plain_16);
		Heltec.display -> drawString(0, (DISPLAY_HEIGHT/2)-8, String(
																																 ((time_month < 10) ? '0' + String(time_month) : String(time_month))
																																 + "/"
																																 + ((time_date < 10) ? '0' + String(time_date) : String(time_date))
																																 + "/"
																																 + time_year
																																 ));
		// print the day of the week at the bottom of the screen
		Heltec.display->setFont(ArialMT_Plain_24);
		Heltec.display -> drawString(0, DISPLAY_HEIGHT-24, String(day_names[time_weekday]));
		
		uint8_t todo_alarm = 0;
		for(int x=0; x < MAX_TODO_ENTRIES; x++){
			if(todos[x].state == OPEN){
				todo_alarm++;
				/* break; // don't break, track how many todos are open */
			}
		}
		if (todo_alarm == 0){ // all todo items are complete!
			Heltec.display->drawXbm((DISPLAY_WIDTH/2)-8, DISPLAY_HEIGHT-16, 16, 16, happyface);
		}
		else{ // there are still todo items that are undone
			// draw sad face
			if (time_seconds % 2 == 0){
				/* Heltec.display->drawXbm((DISPLAY_WIDTH/2)-8, DISPLAY_HEIGHT-16, 16, 16, sadface); */
			  // draw a progress bar instead of the sad face above
				Heltec.display->drawRect((DISPLAY_WIDTH/2)-(MAX_TODO_ENTRIES/2)-1,
																 DISPLAY_HEIGHT-(MAX_TODO_ENTRIES/2)-1,
																 10+2, // add 2 for boarder pixels on each side
																 4+2); // add 2 for boarder pixels on each side
				Heltec.display->fillRect((DISPLAY_WIDTH/2)-(MAX_TODO_ENTRIES/2),
																 DISPLAY_HEIGHT-(MAX_TODO_ENTRIES/2),
																 MAX_TODO_ENTRIES-todo_alarm, // total todos minus the open todos
																 4);
			}
			else{
				// don't draw the sad face
				/* Heltec.display->drawXbm((DISPLAY_WIDTH/2)-8, DISPLAY_HEIGHT-16, 16, 16, sadfaceinvert); */
			}
			if(time_hours >= 7 // if after 7am
				 && time_hours <= 22 // and before midnight
				 && time_minutes == 0 // at the top of the hour
				 && time_seconds == 0
				 && now - last_time_check > 800){ // only make the sound one time per second
				quizzical(); // well, ya gonna do the todos or not?
			}
			/* Heltec.display->drawRect(CURRENT_SELECTION*32, 0, 26, 30); */
		}
		// DO SERIAL STUFF
		if (LAST_SERIAL_INPUT != 0 && (LAST_SERIAL_INPUT_TIME > LAST_SERIAL_DISPLAY_TIME || now - LAST_SERIAL_DISPLAY_TIME > SERIAL_DISPLAY_DELAY)){
			// draw_serial_output single line with auto update
			Serial.println(" "); Serial.println(" "); Serial.println(" ");
			/* serial_cls(); */
			// print the time clock
			Serial.print((time_hours < 10) ? '0' + String(time_hours) : String(time_hours));
			Serial.print(":");
			Serial.println((time_minutes < 10) ? '0' + String(time_minutes) : String(time_minutes));

			Serial.print((time_month < 10) ? '0' + String(time_month) : String(time_month));
			Serial.print("/");
			Serial.print((time_date < 10) ? '0' + String(time_date) : String(time_date));
			Serial.print("/");
			Serial.println(time_year);

			Serial.println(day_names[time_weekday]);

			if (todo_alarm == 0){ // all todo items are complete!
				Serial.println(":D");
			}
			else{ // there are still todo items that are undone
				// draw sad face
				/* Heltec.display->drawXbm((DISPLAY_WIDTH/2)-8, DISPLAY_HEIGHT-16, 16, 16, sadface); */
				// draw a progress bar instead of the sad face above
				Serial.print('[');
				for(int x=0; x < MAX_TODO_ENTRIES; x++){
					if(x >= todo_alarm){
						Serial.print('#');
					}
					else{
						Serial.print(' ');
					}
				}
				Serial.println(']');
			}
			LAST_SERIAL_DISPLAY_TIME = now;
		}
	}
	// end of screen separations, always do the following:
	if (watches[0] != 0){
		draw_watch_logo();
	}
	if ((((now - watches[6])/1000)/3600) >= battery_warning_hours){ // calculation to get hours from stopwatch
		draw_batt_logo();
	}
	draw_countdown_time(); // draw it (and process alarms for it) on every screen
	if (LAST_SERIAL_INPUT != 0){draw_last_serial();}
	if (CURRENT_LAYER == 1){Heltec.display->drawXbm(DISPLAY_WIDTH-8-8-8-1, DISPLAY_HEIGHT-8, 8, 8, star_logo);}
	Heltec.display->display();
}

void oled_print_char_array_range(char array[MAX_ENTRIES][MSG_SIZE], uint8_t selection) {
	int entry_offset_counter = 0;
  long int start = get_start_range_for_selection(MAX_ENTRIES, selection, MAX_SCREEN_LINES);
	for(int entry=start; entry < (start+MAX_SCREEN_LINES); entry++){
		if (entry == selection){ // this is the current selection, so invert it! 
			Heltec.display->setColor(WHITE);
			Heltec.display->fillRect(0, (entry_offset_counter*8)+3, DISPLAY_WIDTH, 8);
			Heltec.display->setColor(BLACK);
		}
		Heltec.display -> drawString(0, entry_offset_counter*8, array[entry]); // is this better notation?
		if (entry == selection){ // done with selection so UN invert
			Heltec.display->setColor(WHITE);
		}
		entry_offset_counter++;
	}
	if (LAST_SERIAL_INPUT != 0 && LAST_SERIAL_INPUT_TIME > LAST_SERIAL_DISPLAY_TIME){
		// draw_serial_output
		serial_cls();
		long int start = get_start_range_for_selection(MAX_ENTRIES, CURRENT_SELECTION, MAX_SERIAL_LINES);
		for(int entry=start; entry < (start+MAX_SERIAL_LINES); entry++){
			char first_character = 32;
			if (entry == CURRENT_SELECTION){
				first_character = '>'; // preceed line with > character
			}
			Serial.print(first_character);
			Serial.println(array[entry]);
			entry_offset_counter++;
		}
		LAST_SERIAL_DISPLAY_TIME = now;
	}

  // It is legal to use array names as constant pointers, and vice versa!
	if (array == menu_options){ // if drawing menu screen, draw the time too
		Heltec.display->setFont(ArialMT_Plain_10);
		// print the numerical time in bottom right of screen
		Heltec.display -> drawString(DISPLAY_WIDTH-(5*6), DISPLAY_HEIGHT-(2*10),
																 String(
																				// String(time_hours) // add a zero instead for military time
																				((time_hours < 10) ? '0' + String(time_hours) : String(time_hours))
																				+ ":"
																				+ ((time_minutes < 10) ? '0' + String(time_minutes) : String(time_minutes))
																				)
																 );
		// print the numerical date in bottom right of screen
		Heltec.display -> drawString(DISPLAY_WIDTH-(8*6), DISPLAY_HEIGHT-(3*10),
																 ((time_month < 10) ? '0' + String(time_month) : String(time_month))
																 + "/"
																 + ((time_date < 10) ? '0' + String(time_date) : String(time_date))
																 + "/"
																 + time_year
																 );
		// print the day of the week at bottom right of screen
		Heltec.display -> drawString(DISPLAY_WIDTH-9*6, DISPLAY_HEIGHT-(2*10), String(day_names[time_weekday]));
	}
}

void oled_print_reference(const char array[MAX_REFERENCE_ENTRIES][REFERENCE_MSG_SIZE], long int selection) {
	long int entry_offset_counter = 0;
  long int start = get_start_range_for_selection(MAX_REFERENCE_ENTRIES, selection, MAX_SCREEN_LINES);
	for(long int entry=start; entry < (start+MAX_SCREEN_LINES); entry++){
		if (entry == selection){ // this is the current selection, so invert it! 
			Heltec.display->setColor(WHITE);
			Heltec.display->fillRect(0, (entry_offset_counter*8)+3, DISPLAY_WIDTH, 8);
			Heltec.display->setColor(BLACK);
		}
		Heltec.display -> drawString(0, entry_offset_counter*8, array[entry]); // is this better notation?
		if (entry == selection){ // done with selection so UN invert
			Heltec.display->setColor(WHITE);
		}
		entry_offset_counter++;
	}
}

void oled_print_calendar(long int selection) {
	long int entry_offset_counter = 0;
  long int start = get_start_range_for_selection(MAX_EVENTS, selection, MAX_SCREEN_LINES);
	for(long int entry=start; entry < (start+MAX_SCREEN_LINES); entry++){
		if (entry == selection){ // this is the current selection, so invert it! 
			Heltec.display->setColor(WHITE);
			Heltec.display->fillRect(0, (entry_offset_counter*8)+3, DISPLAY_WIDTH, 8);
			Heltec.display->setColor(BLACK);
		}
		Heltec.display -> drawString(0, entry_offset_counter*8,
																 String(
																				// String(time_hours) // add a zero instead for military time
																				String(events[entry].month)
																				+ "/"
																				+ String(events[entry].date)
																				+ ":"
																				+ String(events[entry].message)
																				)
																 ); // is this better notation?
		if (entry == selection){ // done with selection so UN invert
			Heltec.display->setColor(WHITE);
		}
		entry_offset_counter++;
	}
}

void display_timer_full_screen(long array [MAX_ENTRIES], long int selection) {
	long now = millis();
	int days = 0;
	int hours = 0;
	int mins =  0;
	int secs =  0;
	char timer_state = ' ';
	if (array[selection] > now){
		hours =(((array[selection] - now)/1000)/3600);
		mins = (((array[selection] - now)/1000) -(3600*hours))/60;
		secs = (((array[selection] - now)/1000) -(3600*hours)-(60*mins));
		timer_state = '+';
	}
	else if (array[selection] == 0){ // time is blank
		timer_state = ' ';
	}
	else if (array[selection] < now){ // time is OVER
		hours =(((now - array[selection])/1000)/3600);
		mins = (((now - array[selection])/1000) - (3600*hours))/60;
		secs = (((now - array[selection])/1000) -(3600*hours)-(60*mins));
		timer_state = '-';
	}
	if (hours > 23){
		days = hours / 24;
		hours = hours - ( days * 24);
	}
	Heltec.display->setFont(ArialMT_Plain_16);
	/* Heltec.display -> drawString(0, 0, String(timer_names[selection])); */
	Heltec.display -> drawStringMaxWidth(0, 0, DISPLAY_WIDTH, String(timer_names[selection]));
	Heltec.display->setFont(ArialMT_Plain_24);
	Heltec.display -> drawString(0, DISPLAY_HEIGHT-24-8, // minus 24 for font height, 8 for countdown timer icon
															 String(
																			/* ((entry < 10) ? '0' + String(entry) : String(entry)) */
																			timer_state
																			/* + String(hours) */
																			/* + ((days > 0) ? String(' ') : String(day_names[time_weekday+days % 7])) */
																			+ String(days)
																			+ ":"
																			+ String(hours)
																			+ ":"
																			+ ((mins < 10) ? '0' + String(mins) : String(mins))
																			+ ":"
																			+ ((secs < 10) ? '0' + String(secs) : String(secs))
																			)
															 );

	  if (LAST_SERIAL_INPUT != 0 && (LAST_SERIAL_INPUT_TIME > LAST_SERIAL_DISPLAY_TIME || now - LAST_SERIAL_DISPLAY_TIME > SERIAL_DISPLAY_DELAY)){
			// draw_serial_output single line with auto update
			Serial.println(" "); Serial.println(" "); Serial.println(" ");
			/* serial_cls(); */
			long int start = get_start_range_for_selection(MAX_ENTRIES, CURRENT_SELECTION, MAX_SERIAL_LINES);
			int weekdays = 0;
			int days = 0;
			int hours = 0;
			int mins =  0;
			int secs =  0;
			char timer_state = ' ';
			if (array[selection] > now){
				hours =(((array[selection] - now)/1000)/3600);
				mins = (((array[selection] - now)/1000) - (3600*hours))/60;
				secs = (((array[selection] - now)/1000) -(3600*hours)-(60*mins));
				timer_state = '+';
			}
			else if (array[selection] == 0){ // time is blank
				timer_state = ' ';
			}
			else if (array[selection] < now){ // time is OVER
				hours =(((now - array[selection])/1000)/3600);
				mins = (((now - array[selection])/1000) - (3600*hours))/60;
				secs = (((now - array[selection])/1000) -(3600*hours)-(60*mins));
				timer_state = '-';
			}
	    if ((time_hours + hours) > 23){ // if days, calculate days and adjust hours
		  	// current time + hours is at least tomorrow
		  	weekdays = (time_hours + hours) / 24;
	    	days = hours / 24;
	    	hours = hours - ( days * 24);
	    }
			Serial.print(timer_state);
			Serial.print((weekdays > 0) ? String(day_names[(time_weekday+weekdays) % 7]) + ":" : "");
			Serial.print(hours);
			Serial.print(":");
			Serial.print(((mins < 10) ? '0' + String(mins) : String(mins)));
			Serial.print(":");
			Serial.print(((secs < 10) ? '0' + String(secs) : String(secs)));
			Serial.print(" ");
			Serial.println(timer_names[selection]);
			LAST_SERIAL_DISPLAY_TIME = now;
		}
}

void oled_print_long_array_range(long array [MAX_ENTRIES], uint8_t selection) {
	// literally just for timers so w/e
	int entry_offset_counter = 0;
	long now = millis();
  long int start = get_start_range_for_selection(MAX_ENTRIES, selection, MAX_SCREEN_LINES);
	for(int entry=start; entry < (start+MAX_SCREEN_LINES); entry++){
		if (entry == selection){ // this is the current selection, so invert it! 
			Heltec.display->setColor(WHITE);
			Heltec.display->fillRect(0, (entry_offset_counter*8)+3, DISPLAY_WIDTH, 8);
			Heltec.display->setColor(BLACK);
		}
		/* string entry_time_string = String(array[entry]); */
	  int weekdays = 0;
	  int days = 0;
		int hours = 0;
  	int mins =  0;
    int secs =  0;
		char timer_state = ' ';
		if (array[entry] > now){
			hours =(((array[entry] - now)/1000)/3600);
			mins = (((array[entry] - now)/1000) - (3600*hours))/60;
			secs = (((array[entry] - now)/1000) -(3600*hours)-(60*mins));
			timer_state = '+';
			}
		else if (array[entry] == 0){ // time is blank
			timer_state = ' ';
		}
		else if (array[entry] < now){ // time is OVER
			hours =(((now - array[entry])/1000)/3600);
			mins = (((now - array[entry])/1000) - (3600*hours))/60;
			secs = (((now - array[entry])/1000) -(3600*hours)-(60*mins));
			timer_state = '-';
			}
	  if ((time_hours + hours) > 23){ // if days, calculate days and adjust hours
			// current time + hours is at least tomorrow
			weekdays = (time_hours + hours) / 24;
	  	days = hours / 24;
	  	hours = hours - ( days * 24);
	  }
		Heltec.display -> drawString(0, entry_offset_counter*8,
																 String(
																				/* ((entry < 10) ? '0' + String(entry) : String(entry)) */
																				timer_state
																				/* + String(hours) */
																			  + ((weekdays > 0) ? String(day_names[(time_weekday+weekdays) % 7]) + ":" : "")
																				+ String(hours)
																				+ ":"
																				+ ((mins < 10) ? '0' + String(mins) : String(mins))
																				+ ":"
																				+ ((secs < 10) ? '0' + String(secs) : String(secs))
																				+ " "
																				+ String(timer_names[entry])
																				)
																 );
		if (entry == selection){ // done with selection so UN invert
			Heltec.display->setColor(WHITE);
		}
		entry_offset_counter++;
	}
	if (LAST_SERIAL_INPUT != 0 && (LAST_SERIAL_INPUT_TIME > LAST_SERIAL_DISPLAY_TIME || now - LAST_SERIAL_DISPLAY_TIME > SERIAL_DISPLAY_DELAY)){
		// draw_serial_output (auto update screen)
		serial_cls();
		/* Serial.println(" "); Serial.println(" "); Serial.println(" "); */
		long int start = get_start_range_for_selection(MAX_ENTRIES, selection, MAX_SERIAL_LINES);
		for(int entry=start; entry < (start+MAX_SERIAL_LINES); entry++){
			char first_character = 32;
			if (entry == CURRENT_SELECTION){
				first_character = '>'; // preceed line with > character
			}
			Serial.print(first_character);
			/* Serial.print(F(notes[i])); */
			int hours = 0;
			int mins =  0;
			int secs =  0;
			char timer_state = ' ';
			if (array[entry] > now){
				hours =(((array[entry] - now)/1000)/3600);
				mins = (((array[entry] - now)/1000) - (3600*hours))/60;
				secs = (((array[entry] - now)/1000) -(3600*hours)-(60*mins));
				timer_state = '+';
			}
			else if (array[entry] == 0){ // time is blank
				timer_state = ' ';
			}
			else if (array[entry] < now){ // time is OVER
				hours =(((now - array[entry])/1000)/3600);
				mins = (((now - array[entry])/1000) - (3600*hours))/60;
				secs = (((now - array[entry])/1000) -(3600*hours)-(60*mins));
				timer_state = '-';
			}
			Serial.print(timer_state);
			Serial.print(hours);
			Serial.print(":");
			Serial.print(((mins < 10) ? '0' + String(mins) : String(mins)));
			Serial.print(":");
			Serial.print(((secs < 10) ? '0' + String(secs) : String(secs)));
			Serial.print(" ");
			Serial.println(timer_names[entry]);
			entry_offset_counter++;
		}
		LAST_SERIAL_DISPLAY_TIME = now;
	}
}

void oled_print_long_array_range_stopwatches(long array [MAX_ENTRIES], uint8_t selection) {
	int entry_offset_counter = 0;
	long now = millis();
  long int start = get_start_range_for_selection(MAX_ENTRIES, selection, MAX_SCREEN_LINES);
	for(int entry=start; entry < (start+MAX_SCREEN_LINES); entry++){
		if (entry == selection){ // this is the current selection, so invert it! 
			Heltec.display->setColor(WHITE);
			Heltec.display->fillRect(0, (entry_offset_counter*8)+3, DISPLAY_WIDTH, 8);
			Heltec.display->setColor(BLACK);
		}
		/* string entry_time_string = String(array[entry]); */
		int hours = 0;
  	int mins =  0;
    int secs =  0;
		if (array[entry] != 0){
			hours =(((now - array[entry])/1000)/3600);
			mins = (((now - array[entry])/1000) - (3600*hours))/60;
			secs = (((now - array[entry])/1000) -(3600*hours)-(60*mins));
			}
		// 	entry_time_string =	String(
		// 														 hours
		// 														 + ":"
		// 														 + ((mins < 10) ? '0' + String(mins) : String(mins))
		// 														 + ":"
		// 														 + ((secs < 10) ? '0' + String(secs) : String(secs))
		// 														 );
		// 	}
		Heltec.display -> drawString(0, entry_offset_counter*8,
																 String(
																				/* ((entry < 10) ? '0' + String(entry) : String(entry)) */
																				/* + " " */
																				/* + String(hours) */
																				String(hours)
																				+ ":"
																				+ ((mins < 10) ? '0' + String(mins) : String(mins))
																				+ ":"
																				+ ((secs < 10) ? '0' + String(secs) : String(secs))
																				+ " "
																				+ String(watch_names[entry])
																				)
																 );
		if (entry == selection){ // done with selection so UN invert
			Heltec.display->setColor(WHITE);
		}
		entry_offset_counter++;
	}
	if (LAST_SERIAL_INPUT != 0 && (LAST_SERIAL_INPUT_TIME > LAST_SERIAL_DISPLAY_TIME || now - LAST_SERIAL_DISPLAY_TIME > SERIAL_DISPLAY_DELAY)){
		// draw_serial_output (auto update screen)
		serial_cls();
		long int start = get_start_range_for_selection(MAX_ENTRIES, selection, MAX_SERIAL_LINES);
		for(int entry=start; entry < (start+MAX_SERIAL_LINES); entry++){
			char first_character = 32;
			if (entry == CURRENT_SELECTION){
				first_character = '>'; // preceed line with > character
			}
			Serial.print(first_character);
			/* Serial.print(F(notes[i])); */
			int hours = 0;
			int mins =  0;
			int secs =  0;
			char timer_state = ' ';
			if (array[entry] > now){
				hours =(((array[entry] - now)/1000)/3600);
				mins = (((array[entry] - now)/1000) - (3600*hours))/60;
				secs = (((array[entry] - now)/1000) -(3600*hours)-(60*mins));
				timer_state = '+';
			}
			else if (array[entry] == 0){ // time is blank
				timer_state = ' ';
			}
			else if (array[entry] < now){ // time is OVER
				hours =(((now - array[entry])/1000)/3600);
				mins = (((now - array[entry])/1000) - (3600*hours))/60;
				secs = (((now - array[entry])/1000) -(3600*hours)-(60*mins));
				timer_state = '-';
			}
			Serial.print(timer_state);
			Serial.print(hours);
			Serial.print(":");
			Serial.print(((mins < 10) ? '0' + String(mins) : String(mins)));
			Serial.print(":");
			Serial.print(((secs < 10) ? '0' + String(secs) : String(secs)));
			Serial.print(" ");
			Serial.println(watch_names[entry]);
			entry_offset_counter++;
		}
		LAST_SERIAL_DISPLAY_TIME = now;
	}
}

void clear_stuff() {
	for(int i=0; i< stuuf_size; i++)
		{
			stuuf[i] = 0;
		}
	stuuf_position = 0;
}

// this is kind of cheating to make a same named function taking different inputs 

long int get_start_range_for_selection(long int MAX_ENTRIES, long int selection, uint8_t max_display_lines) {
	long int start;
	if (MAX_ENTRIES - 1 - selection < max_display_lines / 2){ // selection is near the end of list
		start = MAX_ENTRIES - max_display_lines ; // stop at the last entry in the array
	}
	else if (selection < max_display_lines / 2 ){
		start = 0; // stop at the last entry in the array
	}
	else {
		start =  selection - (max_display_lines / 2); // stop at the last entry in the array
	}
	return start;
}

void reset_selection_and_change_screen(uint8_t new_screen, long int selection){
	CURRENT_SCREEN = new_screen;
	CURRENT_SELECTION = selection;
	draw_screen(CURRENT_SCREEN);
}

int int_concat(int a, int b) 
{ 
    char s1[20]; 
    char s2[20]; 
    // Convert both the integers to string 
    sprintf(s1, "%d", a); 
    sprintf(s2, "%d", b); 
    // Concatenate both strings 
    strcat(s1, s2); 
    // Convert the concatenated string 
    // to integer 
    int c = atoi(s1); 
    // return the formed integer 
    return c; 
} 

void draw_watch_logo(){
	Heltec.display->drawXbm(DISPLAY_WIDTH-8, DISPLAY_HEIGHT-8, 8, 8, watchlogo);
}

void draw_batt_logo(){
	digitalWrite(ledPin, HIGH);
	if (time_seconds % 60 == 0){
		if (now - alert_timeout > 1000){
			annoy();
			alert_timeout =  now;
		}
	}
	if (time_seconds % 2 == 0){
		batterylogo[5] = B01111100;
		batterylogo[6] = B01111100;
		Heltec.display->drawXbm(DISPLAY_WIDTH-8, DISPLAY_HEIGHT-DISPLAY_HEIGHT, 8, 8, batterylogo);
	}
}

void draw_timer_logo(){
  Heltec.display->drawXbm(DISPLAY_WIDTH-8-8, DISPLAY_HEIGHT-8, 8, 8, timerlogochar);
	timerlogochar[timer_logo_rotation] = timerlogochar[timer_logo_rotation] << 1;
	timerlogochar[timer_logo_rotation] = timerlogochar[timer_logo_rotation] | 1;
	if (timerlogochar[timer_logo_rotation] == 255){
		/* timerlogochar[timer_logo_rotation] = 0; // clear previous line */
		timer_logo_rotation++; // increment to next line
		timerlogochar[timer_logo_rotation] = timerlogochar[timer_logo_rotation] | 1; //prime line
		if (timer_logo_rotation > 7){
			timer_logo_rotation = 0;
			timerlogochar[0] = 0;
			timerlogochar[1] = 0;
			timerlogochar[2] = 0;
			timerlogochar[3] = 0;
			timerlogochar[4] = 0;
			timerlogochar[5] = 0;
			timerlogochar[6] = 0;
			timerlogochar[7] = 0;
			timerlogochar[timer_logo_rotation] = timerlogochar[timer_logo_rotation] | 1;
				}
	}
};

void draw_last_serial(){
		Heltec.display->setFont(ArialMT_Plain_10);
		Heltec.display -> drawString(DISPLAY_WIDTH-6, DISPLAY_HEIGHT-12, String(LAST_SERIAL_INPUT));
}

void draw_countdown_time(){
	long now = millis();
	int timer_entry = -1; // start with a null value
	// from the top down, select the first NON zero timer
	for (int e = 0; e < MAX_ENTRIES-1; e++){
		if (timers[e] != 0){
			timer_entry = e;
			break;
		}
	}
	if (timer_entry != -1){ // if found a timer that is set
		if (timers[timer_entry] > now){
			unsigned char hours = 0; // this is an uint8_t, but display prefers unsigned char 
			unsigned char mins = 0;
			unsigned char secs = 0;
			hours =(((timers[timer_entry] - now)/1000)/3600);
			mins = (((timers[timer_entry] - now)/1000) - (3600*hours))/60;
			secs = (((timers[timer_entry] - now)/1000) -(3600*hours)-(60*mins));
			countdown_time_logo[2] = hours;
			countdown_time_logo[2] = countdown_time_logo[2] << 1;
			/* countdown_time_logo[2] = countdown_time_logo[2] | 128; */
			countdown_time_logo[4] = mins;
			countdown_time_logo[4] = countdown_time_logo[4] << 1;
			/* countdown_time_logo[4] = countdown_time_logo[4] | 128; */
			countdown_time_logo[6] = secs;
			countdown_time_logo[6] = countdown_time_logo[6] << 1;
			/* countdown_time_logo[6] = countdown_time_logo[6] | 128; */
			countdown_time_logo[2] = ~countdown_time_logo[2];
			countdown_time_logo[4] = ~countdown_time_logo[4];
			countdown_time_logo[6] = ~countdown_time_logo[6];
			Heltec.display->drawXbm(DISPLAY_WIDTH-8-8, DISPLAY_HEIGHT-8, 8, 8, countdown_time_logo);
		}
		else { // time is up!
			Heltec.display->drawXbm(DISPLAY_WIDTH-8-8, DISPLAY_HEIGHT-8, 8, 8, x_logo);
			// if the alert timeout is more than 1 second, chirp
			// moved this lower, so it alarms on ALL expired countdowntimers
			// if (now - alert_timeout > 1000){
			// 	chirp();
			// 	alert_timeout =  now;
			// }
		}
	}
	for (int e = 0; e < MAX_ENTRIES-1; e++){
		if (timers[e] < now && timers[e] != 0){ // time is OVER
			/* reset_selection_and_change_screen(TIME_DISPLAY_SCREEN, 0); */
			/* Heltec.display->displayOn(); */
			/* setCpuFrequencyMhz(DEFAULT_CPU_FREQ);  */
			if (now - alert_timeout > 1000){
				chirp();
				#ifdef VEXAS_LITE
				LED_blink();
				#endif
				alert_timeout =  now;
				/* moved this to the led blink function */
				/* Heltec.display->invertDisplay(); */
			}
			break;
		}
	}
}


#ifdef VEXAS_LITE
void LED_blink() {
	if (time_seconds % 2 == 0){
		digitalWrite(ledPin, HIGH);
	  Heltec.display->normalDisplay();
	}
	else{
		digitalWrite(ledPin, LOW);
		Heltec.display->invertDisplay();
	}
}
#endif

void cycle_brightness_down() {
	if(brightness == 0){brightness = 255;}
	else if(brightness == 50){brightness = 0;}
	else if(brightness == 150){brightness = 50;}
	else if(brightness == 255){brightness = 150;}
	else {brightness = 150;}
	Heltec.display->setBrightness(brightness);
}

void cycle_brightness_up() {
	if(brightness == 0){brightness = 50;}
	else if(brightness == 50){brightness = 150;}
	else if(brightness == 150){brightness = 255;}
	else if(brightness == 255){brightness = 0;}
	else {brightness = 150;}
	Heltec.display->setBrightness(brightness);
}

void chirp() {
  ledcWriteTone(0,2078);
  delay(30);
  ledcWriteTone(0,4978);
  delay(30);
  ledcWriteTone(0,2078);
  delay(30);
	ledcWrite(0,0);
}

void annoy() {
  ledcWriteTone(0,620);
  delay(100); 
  ledcWriteTone(0,400);
  delay(100); 
	ledcWrite(0,0);
}

void quizzical() {
  ledcWriteTone(0,200);
  delay(75); 
  ledcWriteTone(0,400);
  delay(75); 
  ledcWriteTone(0,620);
  delay(100); 
  ledcWriteTone(0,400);
  delay(100); 
	ledcWrite(0,0);
}

void notone() {
	 ledcWrite(0,0);
}

void reset_alarm_state () {
	Heltec.display->normalDisplay();
#ifdef VEXAS_LITE
	digitalWrite(ledPin, LOW);
#endif
	alert_timeout = 0;
}

void permannoy() {
	ledcWriteTone(0,220);
	delay(100); 
	ledcWriteTone(0,50);
	delay(100); 
}

void serial_dump(){
	Serial.println(" ");
	Serial.println(F("##### BEGIN NOTE DUMP"));
	for ( int i = 0; i < MAX_ENTRIES; i++ ){
		Serial.print("\"");
		Serial.print(F(notes[i]));
		Serial.println("\",");
	}
	Serial.println(F("##### END NOTE DUMP"));
}

void serial_cls(){
	for ( int i = 0; i < SERIAL_DISPLAY_CLS; i++ ){
		Serial.println(" ");
	}
}

void serial_blob_dump(){
	Serial.println(" ");
	Serial.println(F("##### BEGIN BLOB DUMP"));
	Serial.println(" ");
	for (long i = 0; i < BLOB_SIZE; i++ ){
		Serial.print(blob[i]);
	}
	Serial.println(" ");
	Serial.println(F("##### END BLOB DUMP"));
}

void enter_light_sleep() {
				CURRENT_SCREEN = DISPLAY_OFF_SCREEN;
				Heltec.display->displayOff();
				setCpuFrequencyMhz(10);
				/* last_minute_updated = time_minutes; */
				/* delay(2000); */
				// Configure the timer to wake us up!
				/* esp_sleep_enable_timer_wakeup(0.2 * 60L * 1000000L); */
				/* esp_sleep_enable_timer_wakeup(5 * 1000000); // sleep 5 seconds */
				/* esp_sleep_enable_timer_wakeup(5 * 1000000L); */
				// Go to sleep! Zzzz
				/* esp_deep_sleep_start(); */
				// this wipes ram, so it isn't much better than a power switch.
				// this is light sleep
				esp_sleep_enable_timer_wakeup(light_sleep_time_seconds * 1000000); // ESP32 wakes up every X seconds
				esp_light_sleep_start();
}

void exit_light_sleep() {
		reset_selection_and_change_screen(TIME_DISPLAY_SCREEN, 0);
		Heltec.display->displayOn();
		setCpuFrequencyMhz(DEFAULT_CPU_FREQ);
		annoy();
		digitalWrite(ledPin, LOW);
}

void check_events(){
	/* Serial.println("inside check events"); */
	for (int event = 0; event < MAX_EVENTS; event++){
		/* Serial.print("cycling event: "); */
		/* Serial.println(F(events[event].message)); */
		if(time_month == events[event].month && time_date == events[event].date){ // if there's an event today by month/date
			/* Serial.println("valid date event!"); */
			// combine entries into a clock time
			if(    events[event].hour >= 0 // only accept valid clock times
						 && events[event].hour <= 23 
						 && events[event].minute  >= 0
						 && events[event].minute  <= 59
						 ){ // clock time is valid!
				// removed future logic, as check events only happens at 0:00 midnight
				// roll all the timers down
				for (int e = (MAX_ENTRIES-2); e >= 0; e--){ // roll UP all timer names from bottom up, dropping last entry
					// note this will leave the top most entry in position 0 and 1
					// copy the entry into next slot
					for (int m = 0; m < MSG_SIZE; m++){
						timer_names[e+1][m] = timer_names[e][m]; // set current timer name to next timer name
						timers[e+1] = timers[e]; // set current timer to next timer
					}
				}
				// write the event entry into top timer_name field
				for (int m = 0; m < MSG_SIZE; m++){
					timer_names[0][m] = events[event].message[m]; // set first timer name to saved note entry
				}
				// set timer in first entry to event hour:minute
				timers[0] = now
					+ (((long)events[event].hour * 60 * 60 * 1000)
						 +  ((long)events[event].minute * 60) * 1000);
			}
			else{ // else send message reset to a valid time
				// invalid clock time, do nothing.
			}
		}
		else if(events[event].month == 0 && events[event].date == 0 && time_weekday == events[event].weekday){ // this is a weekly event that recurrs on this weekday
			/* Serial.println("valid weekday event!"); */
			// combine entries into a clock time
			if(    events[event].hour >= 0 // only accept valid clock times
						 && events[event].hour <= 23 
						 && events[event].minute  >= 0
						 && events[event].minute  <= 59
						 ){ // clock time is valid!
				// removed future logic, as check events only happens at 0:00 midnight
				// roll all the timers down
				for (int e = (MAX_ENTRIES-2); e >= 0; e--){ // roll UP all timer names from bottom up, dropping last entry
					// note this will leave the top most entry in position 0 and 1
					// copy the entry into next slot
					for (int m = 0; m < MSG_SIZE; m++){
						timer_names[e+1][m] = timer_names[e][m]; // set current timer name to next timer name
						timers[e+1] = timers[e]; // set current timer to next timer
					}
				}
				// write the event entry into top timer_name field
				for (int m = 0; m < MSG_SIZE; m++){
					timer_names[0][m] = events[event].message[m]; // set first timer name to saved note entry
				}
				// set timer in first entry to event hour:minute
				timers[0] = now
					+ (((long)events[event].hour * 60 * 60 * 1000)
						 +  ((long)events[event].minute * 60) * 1000);
			}
			else{ // else send message reset to a valid time
				// invalid clock time, do nothing.
			}
		}
		else {
			/* Serial.println("no events today"); */
		} // this is a weekly event that recurrs on this weekday
	}
}

void check_live_events(){
	/* Serial.println("inside check events"); */
	for (int event = 0; event < MAX_LIVE_EVENTS; event++){
		/* Serial.print("cycling event: "); */
		/* Serial.println(F(events[event].message)); */
		if(time_month == live_events[event].month && time_date == live_events[event].date){ // if there's an event today by month/date
			/* Serial.println("valid date event!"); */
			// combine entries into a clock time
			if(    live_events[event].hour >= 0 // only accept valid clock times
						 && live_events[event].hour <= 23 
						 && live_events[event].minute  >= 0
						 && live_events[event].minute  <= 59
						 ){ // clock time is valid!
				// removed future logic, as check events only happens at 0:00 midnight
				// roll all the timers down
				for (int e = (MAX_ENTRIES-2); e >= 0; e--){ // roll UP all timer names from bottom up, dropping last entry
					// note this will leave the top most entry in position 0 and 1
					// copy the entry into next slot
					for (int m = 0; m < MSG_SIZE; m++){
						timer_names[e+1][m] = timer_names[e][m]; // set current timer name to next timer name
						timers[e+1] = timers[e]; // set current timer to next timer
					}
				}
				// write the event entry into top timer_name field
				for (int m = 0; m < MSG_SIZE; m++){
					timer_names[0][m] = live_events[event].message[m]; // set first timer name to saved note entry
				}
				// set timer in first entry to event hour:minute
				timers[0] = now
					+ (((long)live_events[event].hour * 60 * 60 * 1000)
						 +  ((long)live_events[event].minute * 60) * 1000)
					// minus the current time, bc it's not midnight
					- (((long)time_hours * 60 * 60 * 1000)
						 +  ((long)time_minutes * 60) * 1000);
			}
			else{ // else send message reset to a valid time
				// invalid clock time, do nothing.
			}
		}
		else if(live_events[event].month == 0 && live_events[event].date == 0 && time_weekday == live_events[event].weekday){ // this is a weekly event that recurrs on this weekday
			/* Serial.println("valid weekday event!"); */
			// combine entries into a clock time
			if(    live_events[event].hour >= 0 // only accept valid clock times
						 && live_events[event].hour <= 23 
						 && live_events[event].minute  >= 0
						 && live_events[event].minute  <= 59
						 ){ // clock time is valid!
				// removed future logic, as check events only happens at 0:00 midnight
				// roll all the timers down
				for (int e = (MAX_ENTRIES-2); e >= 0; e--){ // roll UP all timer names from bottom up, dropping last entry
					// note this will leave the top most entry in position 0 and 1
					// copy the entry into next slot
					for (int m = 0; m < MSG_SIZE; m++){
						timer_names[e+1][m] = timer_names[e][m]; // set current timer name to next timer name
						timers[e+1] = timers[e]; // set current timer to next timer
					}
				}
				// write the event entry into top timer_name field
				for (int m = 0; m < MSG_SIZE; m++){
					timer_names[0][m] = live_events[event].message[m]; // set first timer name to saved note entry
				}
				// set timer in first entry to event hour:minute
				timers[0] = now
					+ (((long)live_events[event].hour * 60 * 60 * 1000)
						 +  ((long)live_events[event].minute * 60) * 1000)
					// minus the current time, bc it's not midnight
					- (((long)time_hours * 60 * 60 * 1000)
						 +  ((long)time_minutes * 60) * 1000);
			}
			else{ // else send message reset to a valid time
				// invalid clock time, do nothing.
			}
		}
		else {
			/* Serial.println("no events today"); */
		} // this is a weekly event that recurrs on this weekday
	}
}

void eeprom_write_commit() {
	if (eeprom_has_been_read){ // eeprom has been read since boot, OK to overwrite!
		/* EEPROM.end(); */
		// first write
		long address = 0;
		for ( int i = 0; i < MAX_ENTRIES; i++ ){
			for ( int m = 0; m < MSG_SIZE; m++ ){
				/* EEPROM.write(address, notes[i][m]); */
				EEPROM.put(address, notes[i][m]);
				address++;
			}
		}
		// next, commit
		Heltec.display->clear();
		Heltec.display->drawString(0,  0, "eeprom commit attempt:");
		if(EEPROM.commit()){
			Heltec.display->drawString(0, 10, "eeprom commit SUCCESS!");
		}
		else{
			Heltec.display->drawString(0, 10, "eeprom commit FAIL!");
			Heltec.display->invertDisplay();
		}
		Heltec.display->display();
	}
	else{ // eeprom has not been read this boot! Don't let user overwrite it!
		// but also you can overwrite it with the menu item "eeprom read override"
		Heltec.display->clear();
		Heltec.display->drawString(0, 0, "eeprom write protect active!");
		Heltec.display->drawString(0, 10, "read eeprom before overwriting!");
		Heltec.display->invertDisplay();
		Heltec.display->display();
		delay(1000);
	}
	delay(500);
}

void process_input (char input) {
	if (CURRENT_SCREEN == MENU_SCREEN){ //                           INPUT ON MENU SCREEN
		if(input == 'h' ){
			if(CURRENT_SELECTION == 0 ){reset_selection_and_change_screen(TIME_DISPLAY_SCREEN, 0);}
			else if (CURRENT_SELECTION == 31){time_hours--;if (time_hours == 255){time_hours = 23;}}
			else if (CURRENT_SELECTION == 32){ // SUBTRACTING minutes adjust timers too
				time_minutes--;
			 	if (time_minutes == 255){
					time_minutes = 59;}
				for (int e = 0; e < MAX_ENTRIES-1; e++){
					if (timers[e] > now && timers[e] != 0){ // timer is running, adjust it!
						timers[e] = timers[e] + 1 * 60 * 1000; // No, ADD! one minute from all active unexpired timers
						// clock runs fast, so timer expires 1 minute sooner than it should.
						// correct this by setting the clock BACK a minute, and ADDing an EXTRA minute to the timer
					}
				}
			}
			else if (CURRENT_SELECTION == 33){time_weekday--;if (time_weekday == 255){time_weekday = 6;}}
			else if (CURRENT_SELECTION == 34){time_date--;if (time_date == 0){time_date = MAX_DAYS_IN_THIS_MONTH[time_month];}}
			else if (CURRENT_SELECTION == 38){time_month--;}
			/* else if (CURRENT_SELECTION == 39){MAX_DAYS_IN_THIS_MONTH--;} */
			else if (CURRENT_SELECTION == 40){MAX_SERIAL_LINES--;}
			else{CURRENT_SELECTION = 0;}
		}//                           INPUT ON MENU SCREEN
		else if(input == 'j' || input == KEY_DOWN_ARROW.letter){
			if (CURRENT_SELECTION != MAX_ENTRIES - 1){
				CURRENT_SELECTION++;}
			else {CURRENT_SELECTION = 0;}
		}
		else if(input == 'k' || input == KEY_UP_ARROW.letter){
			if (CURRENT_SELECTION != 0){
				CURRENT_SELECTION--;}
			else {CURRENT_SELECTION = MAX_ENTRIES-1;}
		}//                           INPUT ON MENU SCREEN
		else if(input == '['){
			for (long m = CURRENT_SELECTION-1; m >= 0; m--){
				if (menu_options[m][0] == 32 && menu_options[m][1] == 0){
					CURRENT_SELECTION = m;
					break;
				}
			}
		}//                           INPUT ON MENU SCREEN
		else if(input == ']'){//                           INPUT ON MENU SCREEN
			for (long m = CURRENT_SELECTION+1; m < MAX_ENTRIES-1; m++){
				if (menu_options[m][0] == 32 && menu_options[m][1] == 0){
					CURRENT_SELECTION = m;
					break;
				}
			}
		}//                           INPUT ON MENU SCREEN
		else if(input == 'l'){
			if (CURRENT_SELECTION == 0){reset_selection_and_change_screen(LIST_SCREEN, 0);}
			else if (CURRENT_SELECTION == 1){reset_selection_and_change_screen(TIMER_SCREEN, 0);}
			else if (CURRENT_SELECTION == 2){reset_selection_and_change_screen(TODO_SCREEN, 0);}
			else if (CURRENT_SELECTION == 3){reset_selection_and_change_screen(STOPWATCHES_SCREEN, 0);}
			else if (CURRENT_SELECTION == 4){reset_selection_and_change_screen(CALENDAR_VIEW_SCREEN, 0);}
			else if (CURRENT_SELECTION == 5){
				enter_light_sleep();
			}
			else if (CURRENT_SELECTION == 6){
				reset_selection_and_change_screen(REFERENCE_LIST_SCREEN, 0);
			}
			else if (CURRENT_SELECTION == 10){ // test eeprom write
				long address = 0;
				for ( int i = 0; i < MAX_ENTRIES; i++ ){
					for ( int m = 0; m < MSG_SIZE; m++ ){
						EEPROM.put(address, notes[i][m]);
						address++;
					}
				}
				for ( int i = 0; i < MAX_ENTRIES; i++ ){
					for ( int m = 0; m < MSG_SIZE; m++ ){
						EEPROM.put(address, timer_names[i][m]);
						address++;
					}
				}
			}
			else if (CURRENT_SELECTION == 11){ // eeprom read
				/* if ( EEPROM.read ( 0 ) != 0xff ) // if never written before, will be 0xff */
				// first write
				eeprom_has_been_read = 1;
				long outaddress = 0;
				for ( int i = 0; i < MAX_ENTRIES; i++ ){
					for ( int m = 0; m < MSG_SIZE; m++ ){
						notes[i][m] = EEPROM.read(outaddress);
						outaddress++;
					}
				}
			}//                           INPUT ON MENU SCREEN
			else if (CURRENT_SELECTION == 12){ // eepromWriteCommit
				eeprom_write_commit();
			}
			else if (CURRENT_SELECTION == 13){ // test eeprom commit
				Heltec.display->clear();
				Heltec.display->drawString(0,  0, "eeprom commit attempt:");
				if(EEPROM.commit()){
					Heltec.display->drawString(0, 10, "eeprom commit SUCCESS!");
				}
				else{
					Heltec.display->drawString(0, 10, "eeprom commit FAIL!");
					Heltec.display->invertDisplay();
				}
				Heltec.display->display();
				delay(500);
			}
			else if (CURRENT_SELECTION == 14){Heltec.display->invertDisplay();}
			else if (CURRENT_SELECTION == 15){Heltec.display->normalDisplay();}
			else if (CURRENT_SELECTION == 16){
				chirp();
				#ifdef VEXAS_LITE
					LED_blink();
				#endif
					}
			else if (CURRENT_SELECTION == 17){annoy();}
			else if (CURRENT_SELECTION == 18){notone();}
			else if (CURRENT_SELECTION == 19){permannoy();}
			else if (CURRENT_SELECTION == 20){ // test read 
				long outaddress = 0;
				for ( int i = 0; i < MAX_ENTRIES; i++ ){
					for ( int m = 0; m < MSG_SIZE; m++ ){
						notes[i][m] = EEPROM.read(outaddress);
						outaddress++;
					}
				}
				for ( int i = 0; i < MAX_ENTRIES; i++ ){
					for ( int m = 0; m < MSG_SIZE; m++ ){
						timer_names[i][m] = EEPROM.read(outaddress);
						outaddress++;
					}
				}
			}
			else if (CURRENT_SELECTION == 21){Heltec.display->mirrorScreen();}
			else if (CURRENT_SELECTION == 22){Heltec.display->setBrightness(0);}
			else if (CURRENT_SELECTION == 23){Heltec.display->setBrightness(50);}
			else if (CURRENT_SELECTION == 24){Heltec.display->setBrightness(150);}
			else if (CURRENT_SELECTION == 25){Heltec.display->setBrightness(255);}
			else if (CURRENT_SELECTION == 26){setCpuFrequencyMhz(240);DEFAULT_CPU_FREQ = 240;}
			else if (CURRENT_SELECTION == 27){setCpuFrequencyMhz(80);DEFAULT_CPU_FREQ = 80;}
			else if (CURRENT_SELECTION == 28){setCpuFrequencyMhz(40);DEFAULT_CPU_FREQ = 40;} // good balance
			else if (CURRENT_SELECTION == 29){setCpuFrequencyMhz(10);DEFAULT_CPU_FREQ = 10;}
			/* else if (CURRENT_SELECTION == 30){} */
			else if (CURRENT_SELECTION == 31){time_hours++;
				if (time_hours > 23){time_hours = 0;}}
			else if (CURRENT_SELECTION == 32){
				time_minutes++;
				if (time_minutes > 59){time_minutes = 0;}
				for (int e = 0; e < MAX_ENTRIES-1; e++){
					if (timers[e] > now && timers[e] != 0){ // timer is running, adjust it!
						timers[e] = timers[e] - 1 * 60 * 1000; // subtract one minute from all active unexpired timers
						// clock runs fast, so timer expires 1 minute sooner than it should.
						// correct this by setting the clock FORWARD a minute, and SUBTRACTING an EXTRA minute from the timer
					}
				}
			}
			else if (CURRENT_SELECTION == 33){time_weekday++;
				if (time_weekday > 6){time_weekday = 0;}}
			else if (CURRENT_SELECTION == 34){time_date++;
				if (time_date > MAX_DAYS_IN_THIS_MONTH[time_month]){time_date = 1;}}
			else if (CURRENT_SELECTION == 36 && LAST_INPUT == 'l'){ESP.restart();} // press it twice!
			else if (CURRENT_SELECTION == 37){serial_dump();}
			else if (CURRENT_SELECTION == 38){time_month++;}
			/* else if (CURRENT_SELECTION == 39){MAX_DAYS_IN_THIS_MONTH++;} */
			else if (CURRENT_SELECTION == 40){MAX_SERIAL_LINES++;}
			else if (CURRENT_SELECTION == 41){reset_selection_and_change_screen(BLOB_SCREEN, 0);}
			else if (CURRENT_SELECTION == 42){serial_blob_dump();}
			else if (CURRENT_SELECTION == 43){ // wipe blob
				for(long i=0; i< BLOB_SIZE; i++) {
						blob[i] = 0;
					}
				blob_position = 0;
			}
			else if (CURRENT_SELECTION == 44){check_live_events();}
			else if (CURRENT_SELECTION == 45){check_events();}
			else if (CURRENT_SELECTION == 46){time_seconds = 0;now = millis();last_time_check = millis();}
			// menu item 47 is blank
			else if (CURRENT_SELECTION == 48){eeprom_has_been_read = 1;}
		}//                           INPUT ON MENU SCREEN
	}
	else if (CURRENT_SCREEN == LIST_SCREEN){ //                      INPUT ON LIST SCREEN
		if(input == 'h' ){
			reset_selection_and_change_screen(MENU_SCREEN, 0);
		}
		else if(input == 'j'){
			if (CURRENT_SELECTION != MAX_ENTRIES - 1){CURRENT_SELECTION++;}
		}
		else if(input == 'k'){
			if (CURRENT_SELECTION != 0){CURRENT_SELECTION--;}
		}
		else if(input == '['){
			for (long m = CURRENT_SELECTION-1; m >= 0; m--){
				if (notes[m][0] == 32 && notes[m][1] == 0){
					CURRENT_SELECTION = m;
					break;
				}
			}
		}
		else if(input == ']'){
			for (long m = CURRENT_SELECTION+1; m < MAX_ENTRIES-1; m++){
				if (notes[m][0] == 32 && notes[m][1] == 0){
					CURRENT_SELECTION = m;
					break;
				}
			}
		}
		else if(input == 'l' ){
			string_entry_ptr = &notes[CURRENT_SELECTION];
			EDIT_SELECTION = CURRENT_SELECTION;
			LAST_SCREEN = CURRENT_SCREEN;
			reset_selection_and_change_screen(STRING_EDIT_SCREEN, 0);
		}
		else if(input == ';' ){ // does the same thing happen with ;?
			string_entry_ptr = &notes[CURRENT_SELECTION];
			EDIT_SELECTION = CURRENT_SELECTION;
			LAST_SCREEN = CURRENT_SCREEN;
			reset_selection_and_change_screen(STRING_EDIT_SCREEN, 0);
		}
		else if(input == 'a'){
			for (int m = 0; m < (MSG_SIZE-1); m++){ // check for null forwards from begin of entry
				if (notes[CURRENT_SELECTION][m] == 0){ // if this entry spot is null
					if (m != 0){
						edit_position = m;
						break;
					}
				}
			}
			string_entry_ptr = &notes[CURRENT_SELECTION];
			EDIT_SELECTION = CURRENT_SELECTION;
			LAST_SCREEN = CURRENT_SCREEN;
			reset_selection_and_change_screen(STRING_EDIT_SCREEN, 0);
		}
		else if(input == 'e'){ // does the same thing happen with e?
			for (int m = 0; m < (MSG_SIZE-1); m++){ // check for null forwards from begin of entry
				if (notes[CURRENT_SELECTION][m] == 0){ // if this entry spot is null
					if (m != 0){
						edit_position = m;
						break;
					}
				}
			}
			string_entry_ptr = &notes[CURRENT_SELECTION];
			EDIT_SELECTION = CURRENT_SELECTION;
			LAST_SCREEN = CURRENT_SCREEN;
			reset_selection_and_change_screen(STRING_EDIT_SCREEN, 0);
		}
		else if(input == 'u' && CURRENT_SELECTION != MAX_ENTRIES - 1){ // move entry down UNLESS LAST
			// it makes sense to make this movement roll over, I can do that later
			// move destination sleection to holding
			for (int m = 0; m < MSG_SIZE; m++){
				entrycopyspace[m] = notes[CURRENT_SELECTION + 1][m];
			}
			// move current to destination
			// move holding back to previous locatoin
			for (int m = 0; m < MSG_SIZE; m++){
				notes[CURRENT_SELECTION + 1][m] = notes[CURRENT_SELECTION][m];
				notes[CURRENT_SELECTION][m] = entrycopyspace[m];
			}
			CURRENT_SELECTION++;
		}
		else if(input == 'i' && CURRENT_SELECTION != 0){  // move entry up UNLESS FIRST
			// it makes sense to make this movement roll over, I can do that later
			// move destination sleection to holding
			for (int m = 0; m < MSG_SIZE; m++){
				entrycopyspace[m] = notes[CURRENT_SELECTION - 1][m];
			}
			// move current to destination
			// move holding back to previous locatoin
			for (int m = 0; m < MSG_SIZE; m++){
				notes[CURRENT_SELECTION - 1][m] = notes[CURRENT_SELECTION][m];
				notes[CURRENT_SELECTION][m] = entrycopyspace[m];
			}
			CURRENT_SELECTION--;
		}
		else if(input == 'o'){ // move last entry up to current selection, blank, & edit
			// move second to last position into last & ascend until current selection
			for (int e = (MAX_ENTRIES-2); e > CURRENT_SELECTION; e--){
				// copy the entry into next slot
				for (int m = 0; m < MSG_SIZE; m++){
					notes[e+1][m] = notes[e][m]; // set next entry to this entry
				}
			}
			// (next line) nah, make o vim-like so it opens a blank line BELOW current entry
			CURRENT_SELECTION++;
			// (last line) nah, make o vim-like so it opens a blank line BELOW current entry
			// blank current selection
			for (int m = 0; m < MSG_SIZE; m++){
				notes[CURRENT_SELECTION][m] = 0; // set to null (blank)
			}
			notes[CURRENT_SELECTION][0] = 32; // set first position to space
			// begin editing the current selection
			string_entry_ptr = &notes[CURRENT_SELECTION];
			EDIT_SELECTION = CURRENT_SELECTION;
			LAST_SCREEN = CURRENT_SCREEN;
			reset_selection_and_change_screen(STRING_EDIT_SCREEN, 0);
		}
		else if(input == 'd' && LAST_INPUT == 'd'){ // was d pressed twice?
			// blank entry, move it to the bottom of entry list
			// before delete, copy current entry into entry register
			for (int m = 0; m < MSG_SIZE; m++){
				entrycopyspace[m] = notes[CURRENT_SELECTION][m]; // set next entry to this entry
			}
			// copy next selection to current until end of entry list,
			for (int e = CURRENT_SELECTION; e < MAX_ENTRIES-1; e++){
				for (int m = 0; m < MSG_SIZE; m++){
					notes[e][m] = notes[e+1][m]; // set next entry to this entry
				}
			}
			// blank out the last entry
			for (int m = 0; m < MSG_SIZE; m++){
				notes[MAX_ENTRIES-1][m] = 0; // set to space (blank)
			}
			notes[MAX_ENTRIES-1][0] = 32; // set first position to space (blank)
		}
		else if(input == 'y'){ // YANK current entry into entry register
			// before delete, copy current entry into copy space
			for (int m = 0; m < MSG_SIZE; m++){
				entrycopyspace[m] = notes[CURRENT_SELECTION][m]; // set next entry to this entry
			}
		}
		else if(input == 't'){ // make a new timer with this as a note for the timer name
			// copy current note to copy space
			for (int m = 0; m < MSG_SIZE; m++){
				entrycopyspace[m] = notes[CURRENT_SELECTION][m]; // copy current note into copy space
			}
			// roll all the timers down
			for (int e = (MAX_ENTRIES-2); e >= 0; e--){ // roll UP all timer names from bottom up, dropping last entry
				// note this will leave the top most entry in position 0 and 1
				// copy the entry into next slot
				for (int m = 0; m < MSG_SIZE; m++){
					timer_names[e+1][m] = timer_names[e][m]; // set current timer name to next timer name
					timers[e+1] = timers[e]; // set current timer to next timer
				}
			}
			// write the copy space entry into top timer_name field
			for (int m = 0; m < MSG_SIZE; m++){
				timer_names[0][m] = entrycopyspace[m]; // set first timer name to saved note entry
			}
			// now to pick a time for this entry;
			EDIT_SELECTION = 0;
			reset_selection_and_change_screen(TIMER_SET_SCREEN, 0);
		}
		else if(input == 's'){ // make a new timer at snooze minutes with this note as timer name
			// copy current note to copy space
			for (int m = 0; m < MSG_SIZE; m++){
				entrycopyspace[m] = notes[CURRENT_SELECTION][m]; // copy current note into copy space
			}
			// roll all the timers down
			for (int e = (MAX_ENTRIES-2); e >= 0; e--){ // roll UP all timer names from bottom up, dropping last entry
				// note this will leave the top most entry in position 0 and 1
				// copy the entry into next slot
				for (int m = 0; m < MSG_SIZE; m++){
					timer_names[e+1][m] = timer_names[e][m]; // set current timer name to next timer name
					timers[e+1] = timers[e]; // set current timer to next timer
				}
			}
			// write the copy space entry into top timer_name field
			for (int m = 0; m < MSG_SIZE; m++){
				timer_names[0][m] = entrycopyspace[m]; // set first timer name to saved note entry
			}
			// now snooze the timer and switch screens
			// EDIT_SELECTION = 0; // we're not editing this time;
			timers[0] = now + 1000 + snooze_time * 60 * 1000; // set newlycreated timer to +snooze minutes
			// added 2 seconds ^ to snooze timer so it wouldn't start at 4:59 all the time
			// flip screen to timer screen with new timer selected
			reset_selection_and_change_screen(TIMER_SCREEN, 0);
		}
		else if(input == 'p' && CURRENT_SELECTION < MAX_ENTRIES-2){ // technically, this can be max_entries-1, but, this is safer
			// PUT entry register deleted entry into next entry
			// move second to last position into last & ascend until current selection
			for (int e = (MAX_ENTRIES-2); e > CURRENT_SELECTION; e--){
				// copy the entry into next slot
				for (int m = 0; m < MSG_SIZE; m++){
					notes[e+1][m] = notes[e][m]; // set next entry to this entry
				}
			}
			// (next line) nah, make o vim-like so it opens a blank line BELOW current entry
			CURRENT_SELECTION++;
			// populate current selection with the entry copy space 
			for (int m = 0; m < MSG_SIZE; m++){
				notes[CURRENT_SELECTION][m] = entrycopyspace[m]; // set next entry to this entry
			}
		}
		else if(input == 'v'){ // straight note view large screen
			LAST_SCREEN = CURRENT_SCREEN;
			reset_selection_and_change_screen(LARGE_STRING_VIEW_SCREEN, CURRENT_SELECTION);
		}
		else if(input == 8 && LAST_INPUT == 8){ // backspace pressed TWICE
			for (int m = 0; m < MSG_SIZE; m++){
				entrycopyspace[m] = notes[CURRENT_SELECTION][m]; // set next entry to this entry
			}
			for (int m = 0; m < MSG_SIZE-1; m++){
				notes[CURRENT_SELECTION][m] = 0; // set to null
			}
			notes[CURRENT_SELECTION][0] = 32; // set first position to space (blank)
		}
		else if(input == '/' ){ // search?
			// Blank out the search string
			for (int m = 0; m < MSG_SIZE-1; m++){
				search_string[m] = 0; // set to null
			}
			// I don't think I need to prep this with a space below
			/* search_string[0] = 32; // set first position to space (blank) */
			LAST_SCREEN = CURRENT_SCREEN;
			EDIT_SELECTION = CURRENT_SELECTION;
			reset_selection_and_change_screen(LIST_SEARCH_SCREEN, 0);
		}
	}
	else if (CURRENT_SCREEN == LIST_SEARCH_SCREEN){ //                     INPUT ON LIST SEARCH STRING SCREEN
		if(input == 8){ // backspace
			edit_position--;
			if (edit_position == 255){ // if backspace on zero position rolls over, reset it
				edit_position = 0;
			}
			search_string[edit_position] = 0; // set to space (this is not standard, but fine I think.
			/* (*string_entry_ptr)[edit_position] = 0; // nah backspace it to 0 */
		}
		else if(input == 13){ // return
			edit_position = 0;
			/* reset_selection_and_change_screen(LIST_SCREEN, EDIT_SELECTION); */
			reset_selection_and_change_screen(LIST_SCREEN, EDIT_SELECTION);
		}
		else if(input == 18){ // left arrow
			if (CURRENT_SELECTION != 0){CURRENT_SELECTION--;}
		}
		else if(input == 19){ // right arrow
			// Don't let me arrow past the message size OR over a null
			if (CURRENT_SELECTION != sizeof(notes) - 1){CURRENT_SELECTION++;}
		}
		else{
			if (input > 31){ // if it's a printable character
				search_string[edit_position] = input;
				edit_position++;
				if (edit_position == MSG_SIZE-1){// no, don't get to the end position
					edit_position--;
				}
			}
		}
	}
	else if (CURRENT_SCREEN == REFERENCE_LIST_SCREEN){ //                      INPUT ON REFERENCE SCREEN
		if(input == 'h' ){
			reset_selection_and_change_screen(MENU_SCREEN, 6);
		}
		else if(input == 'j'){
			if (CURRENT_SELECTION != MAX_REFERENCE_ENTRIES - 1){CURRENT_SELECTION++;}
		}
		else if(input == 'k'){
			if (CURRENT_SELECTION != 0){CURRENT_SELECTION--;}
		}
		else if(input == '['){
			for (long m = CURRENT_SELECTION-1; m >= 0; m--){
				if (REFERENCE[m][0] == 32 && REFERENCE[m][1] == 0){
					CURRENT_SELECTION = m;
					break;
				}
			}
		}
		else if(input == ']'){
			for (long m = CURRENT_SELECTION+1; m < MAX_REFERENCE_ENTRIES-1; m++){
				if (REFERENCE[m][0] == 32 && REFERENCE[m][1] == 0){
					CURRENT_SELECTION = m;
					break;
				}
			}
		}
		else if(input == 'l' ){
			LAST_SCREEN = CURRENT_SCREEN;
			reset_selection_and_change_screen(LONG_STRING_VIEW_SCREEN, CURRENT_SELECTION);
		}
		else if(input == '/' ){ // search?
			// Blank out the search string
			for (int m = 0; m < MSG_SIZE-1; m++){
				search_string[m] = 0; // set to null
			}
			// I don't think I need to prep this with a space below
			/* search_string[0] = 32; // set first position to space (blank) */
			EDIT_SELECTION = CURRENT_SELECTION;
			reset_selection_and_change_screen(REFERENCE_SEARCH_SCREEN, 0);
		}
		else if(input == 'y'){ // YANK reference into entry register
			for (int m = 0; m < MSG_SIZE; m++){
				entrycopyspace[m] = REFERENCE[CURRENT_SELECTION][m]; // set next entry to this entry
			}
		}
	}
	else if (CURRENT_SCREEN == CALENDAR_VIEW_SCREEN){ //                      INPUT ON CALENDAR SCREEN
		if(input == 'h' ){
			reset_selection_and_change_screen(MENU_SCREEN, 4);
		}
		else if(input == 'j'){
			if (CURRENT_SELECTION != MAX_EVENTS - 1){CURRENT_SELECTION++;}
		}
		else if(input == 'k'){
			if (CURRENT_SELECTION != 0){CURRENT_SELECTION--;}
		}
		// else if(input == 'l' ){
		// 	LAST_SCREEN = CURRENT_SCREEN;
		// 	reset_selection_and_change_screen(LONG_STRING_VIEW_SCREEN, CURRENT_SELECTION);
		// }
		else if(input == 'y'){ // YANK reference into entry register
			for (int m = 0; m < MSG_SIZE; m++){
				entrycopyspace[m] = events[CURRENT_SELECTION].message[m]; // set next entry to this entry
			}
		}
	}
	else if (CURRENT_SCREEN == REFERENCE_SEARCH_SCREEN){ //                     INPUT ON REFERENCE SEARCH STRING SCREEN
		// normally this would match the other search screen, but it should react differently
		if(input == 8){ // backspace
			edit_position--;
			if (edit_position == 255){ // if backspace on zero position rolls over, reset it
				edit_position = 0;
			}
			search_string[edit_position] = 0; // set to space (this is not standard, but fine I think.
			/* (*string_entry_ptr)[edit_position] = 0; // nah backspace it to 0 */
		}
		else if(input == 13){ // return
			edit_position = 0;
			/* reset_selection_and_change_screen(LIST_SCREEN, EDIT_SELECTION); */
			reset_selection_and_change_screen(REFERENCE_LIST_SCREEN, EDIT_SELECTION);
		}
		else if(input == 18){ // left arrow
			if (CURRENT_SELECTION != 0){CURRENT_SELECTION--;}
		}
		else if(input == 19){ // right arrow
			// Don't let me arrow past the message size OR over a null
			if (CURRENT_SELECTION != MAX_REFERENCE_ENTRIES - 1){CURRENT_SELECTION++;}
		}
		else{
			if (input > 31){ // if it's a printable character
				search_string[edit_position] = input;
				edit_position++;
				if (edit_position == REFERENCE_MSG_SIZE-1){// no, don't get to the end position
					edit_position--;
				}
			}
		}
	}
	else if (CURRENT_SCREEN == DISPLAY_OFF_SCREEN){ //               INPUT ON DISPLAY OFF SCREEN
		/* reset_selection_and_change_screen(TIME_DISPLAY_SCREEN, 0); */
		/* Heltec.display->displayOn(); */
		setCpuFrequencyMhz(DEFAULT_CPU_FREQ);
		exit_light_sleep();
	}
	else if (CURRENT_SCREEN == LONG_STRING_VIEW_SCREEN){ //               INPUT ON STRING VIEW SCREEN
		if(input == 'j'){
			if (CURRENT_SELECTION != MAX_REFERENCE_ENTRIES - 1){CURRENT_SELECTION++;}
		}
		else if(input == 'k'){
			if (CURRENT_SELECTION != 0){CURRENT_SELECTION--;}
		}
		else if(input == 'h'){
			reset_selection_and_change_screen(LAST_SCREEN, CURRENT_SELECTION);
		}
		else{
			// TODO this is a bug that might be related to a defect/damage on MY heltec board;
			// if it bails on any additional keypresses, it will go to the screen, then more keys
			// will trigger, and it will dump you back out. this error occurs sometimes when
			// editing strings, it will press the button you pressed, then process multiple additional keys
			// I don't want to hack a way around it until I confirm it's just my hardware, and not an actual bug
		}
	}
	else if (CURRENT_SCREEN == LARGE_STRING_VIEW_SCREEN){ //               INPUT ON LARGE STRING NOTE VIEW SCREEN
		if(input == 'j'){
			if (CURRENT_SELECTION != sizeof(notes) - 1){CURRENT_SELECTION++;}
		}
		else if(input == 'k'){
			if (CURRENT_SELECTION != 0){CURRENT_SELECTION--;}
		}
		else if(input == 'h'){
			reset_selection_and_change_screen(LAST_SCREEN, CURRENT_SELECTION);
		}
		else{
		}
	}
	else if (CURRENT_SCREEN == TIME_DISPLAY_SCREEN){ //               INPUT ON TIME_DISPLAY_SCREEN
		if(input == 'j'){
			cycle_brightness_down();
		}
		else if(input == 'k'){
			cycle_brightness_up();
		}
		else if(input == 'l'){
			reset_selection_and_change_screen(MENU_SCREEN, 0);
		}
		else if(input == 'o'){ // quick note entry
			CURRENT_SELECTION = 0;
			// move second to last position into last & ascend until current selection
			for (int e = (MAX_ENTRIES-2); e > CURRENT_SELECTION; e--){
				// copy the entry into next slot
				for (int m = 0; m < MSG_SIZE; m++){
					notes[e+1][m] = notes[e][m]; // set next entry to this entry
				}
			}
			// (next line) nah, make o vim-like so it opens a blank line BELOW current entry
			CURRENT_SELECTION++;
			// (last line) nah, make o vim-like so it opens a blank line BELOW current entry
			// blank current selection
			for (int m = 0; m < MSG_SIZE; m++){
				notes[CURRENT_SELECTION][m] = 0; // set to null (blank)
			}
			notes[CURRENT_SELECTION][0] = 32; // set first position to space
			// begin editing the current selection
			string_entry_ptr = &notes[CURRENT_SELECTION];
			EDIT_SELECTION = CURRENT_SELECTION;
			LAST_SCREEN = CURRENT_SCREEN;
			reset_selection_and_change_screen(STRING_EDIT_SCREEN, 0);
		}
		else if(input == 't'){ // jump to first expired timer
			int timer_entry = -1; // start with a null value
			// from the top down, select the first NON zero timer
			for (int e = 0; e < MAX_ENTRIES-1; e++){
				if (timers[e] != 0 && timers[e] < now){ // first expired timer!
					timer_entry = e;
					break;
				}
			}
			LAST_SCREEN = CURRENT_SCREEN;
			if(timer_entry != -1){ // If no expired timers were found
				reset_selection_and_change_screen(LARGE_TIMER_VIEW_SCREEN, timer_entry);
			}
			else{reset_selection_and_change_screen(TIMER_SCREEN, 0);}
		}
		else if(input == 'p'){ // shortcut to sleeP
			enter_light_sleep();
		}
		else if(input == KEY_BATTRST.letter){
			watches[6] = millis();
			Heltec.display->invertDisplay();
			delay(250);
			Heltec.display->normalDisplay();
			light_sleep_time_seconds = light_sleep_time_seconds_default;
			digitalWrite(ledPin, LOW);
			eeprom_written_this_battery_reset = 0;
		} //                                                INPUT ON TIME_DISPLAY_SCREEN
	}
	else if (CURRENT_SCREEN == TODO_SCREEN){ //               INPUT ON TODO screen
		if(input == 'h' ){
			reset_selection_and_change_screen(MENU_SCREEN, 2);
		}
		else if(input == 'j' ){
			if (CURRENT_SELECTION != MAX_TODO_ENTRIES - 1){CURRENT_SELECTION++;}
		}
		else if(input == 'k' ){
			if (CURRENT_SELECTION != 0){CURRENT_SELECTION--;}
		}
		else if(input == 'l' ){
			// toggle todo state
			if (todos[CURRENT_SELECTION].state == CLOSED){
				todos[CURRENT_SELECTION].state = OPEN;
				/* todos[CURRENT_SELECTION].date_closed = 0; // set a null date so it will always alarm */
			}
			else if (todos[CURRENT_SELECTION].state == OPEN){
				todos[CURRENT_SELECTION].state = CLOSED;
				/* todos[CURRENT_SELECTION].date_closed = time_date; */
			}
		}
	}
	else if (CURRENT_SCREEN == STOPWATCHES_SCREEN){ //               INPUT ON STOPWATCHES SCREEN
		if(input == 'h' ){
			reset_selection_and_change_screen(MENU_SCREEN, 3);
		}
		else if(input == 'j' ){
			if (CURRENT_SELECTION != MAX_ENTRIES - 1){CURRENT_SELECTION++;}
		}
		else if(input == 'k' ){
			if (CURRENT_SELECTION != 0){CURRENT_SELECTION--;}
		}
		else if(input == 'l' ){
			if (watches[CURRENT_SELECTION] == 0){ // if stopwatch is NOT set
				watches[CURRENT_SELECTION] = now;
				LAST_SCREEN = CURRENT_SCREEN;
				EDIT_SELECTION = CURRENT_SELECTION;
				string_entry_ptr = &watch_names[CURRENT_SELECTION];
				reset_selection_and_change_screen(STRING_EDIT_SCREEN, CURRENT_SELECTION);
			}
			else{
				watches[CURRENT_SELECTION] = 0;
			}
		}
		else if(input == 's'){ // snooze for current time watch, or start new for current stopwatch
			if (watches[CURRENT_SELECTION] == 0){ // if stopwatch is NOT set, start it without string entry
				watches[CURRENT_SELECTION] = now;
			}
			else{ // add snooze_time minutes (technically, subtract)
				watches[CURRENT_SELECTION] = watches[CURRENT_SELECTION] - (snooze_time * 60 * 1000);
			}
		}
		else if(input == 'w'){ // add hour for current time watch, or start new for current stopwatch
			if (watches[CURRENT_SELECTION] == 0){ // if stopwatch is NOT set, start it without string entry
				watches[CURRENT_SELECTION] = now;
			}
			else{ // add snooze_time minutes (technically, subtract)
				watches[CURRENT_SELECTION] = watches[CURRENT_SELECTION] - (big_snooze_time * 60 * 1000);
			}
		}
		else if(input == 'x'){ // add hour for current time watch, or start new for current stopwatch
			if (watches[CURRENT_SELECTION] == 0){ // if stopwatch is NOT set, start it without string entry
				watches[CURRENT_SELECTION] = now;
			}
			else{ // add snooze_time minutes (technically, subtract)
				watches[CURRENT_SELECTION] = watches[CURRENT_SELECTION] - (small_snooze_time * 60 * 1000);
			}
		}
		else if(input == 'y'){ // YANK stopwatch name into entry register
			for (int m = 0; m < MSG_SIZE; m++){
				entrycopyspace[m] = watch_names[CURRENT_SELECTION][m]; // set next entry to this entry
			}
		}
		else if(input == 8 && LAST_INPUT == 8){ // backspace pressed TWICE
			for (int m = 0; m < MSG_SIZE-1; m++){
				watch_names[CURRENT_SELECTION][m] = 0; // set to null
			}
			watch_names[CURRENT_SELECTION][0] = 32; // set first position to space (blank)
		}
		else if(input == 'p' && CURRENT_SELECTION < MAX_ENTRIES){ // PUT entry register deleted entry into next entry
			// populate current selection with the entry register
			for (int m = 0; m < MSG_SIZE; m++){
				watch_names[CURRENT_SELECTION][m] = entrycopyspace[m]; // set next entry to this entry
			}
		}
		else if(input == 'u' && CURRENT_SELECTION != MAX_ENTRIES - 1){ // move entry down UNLESS LAST
			timercopyspace = watches[CURRENT_SELECTION+1]; // set holding slot to next selection down
			watches[CURRENT_SELECTION+1] = watches[CURRENT_SELECTION]; // fill next seledction down with current selection
			watches[CURRENT_SELECTION] = timercopyspace; // fill current selection with holding slot
			for (int m = 0; m < MSG_SIZE; m++){
				entrycopyspace[m] = watch_names[CURRENT_SELECTION + 1][m];
			}
			for (int m = 0; m < MSG_SIZE; m++){
				watch_names[CURRENT_SELECTION + 1][m] = watch_names[CURRENT_SELECTION][m];
				watch_names[CURRENT_SELECTION][m] = entrycopyspace[m];
			}
			CURRENT_SELECTION++;
		}
		else if(input == 'i' && CURRENT_SELECTION != 0){  // move entry up UNLESS FIRST
			timercopyspace = watches[CURRENT_SELECTION-1]; // set holding slot to next selection down
			watches[CURRENT_SELECTION-1] = watches[CURRENT_SELECTION]; // fill next seledction down with current selection
			watches[CURRENT_SELECTION] = timercopyspace; // fill current selection with holding slot
			for (int m = 0; m < MSG_SIZE; m++){
				entrycopyspace[m] = watch_names[CURRENT_SELECTION - 1][m];
			}
			for (int m = 0; m < MSG_SIZE; m++){
				watch_names[CURRENT_SELECTION - 1][m] = watch_names[CURRENT_SELECTION][m];
				watch_names[CURRENT_SELECTION][m] = entrycopyspace[m];
			}
			CURRENT_SELECTION--;
		}
	}
	else if (CURRENT_SCREEN == TIMER_SCREEN){ //                     INPUT ON TIMER SCREEN
		if(input == 'h' ){
			reset_selection_and_change_screen(MENU_SCREEN, 1);
		}
		else if(input == 'j' ){
			/* CURRENT_SELECTION++; */
			if (CURRENT_SELECTION != MAX_ENTRIES - 1){CURRENT_SELECTION++;}
			draw_screen(CURRENT_SCREEN);
		}
		else if(input == 'k' ){
			/* CURRENT_SELECTION--; */
			if (CURRENT_SELECTION != 0){CURRENT_SELECTION--;}
			draw_screen(CURRENT_SCREEN);
		}
		else if(input == 'l' ){
			EDIT_SELECTION = CURRENT_SELECTION;
			reset_selection_and_change_screen(TIMER_SET_SCREEN, 0);
		}
		else if(input == 'q'){ // blank out the current countdown timer
			timers[CURRENT_SELECTION] = 0;
			reset_alarm_state();
		}
		else if(input == 'p' && CURRENT_SELECTION < MAX_ENTRIES){ // PUT entry register deleted entry into next entry
			// populate current selection with the entry register
			for (int m = 0; m < MSG_SIZE; m++){
				timer_names[CURRENT_SELECTION][m] = entrycopyspace[m]; // set next entry to this entry
			}
		}
		else if(input == 8 && LAST_INPUT == 8){ // backspace pressed TWICE
			for (int m = 0; m < MSG_SIZE-1; m++){
				timer_names[CURRENT_SELECTION][m] = 0; // set to null
			}
			timer_names[CURRENT_SELECTION][0] = 32; // set first position to space (blank)
		}
		else if(input == 's'){ // snooze the timer by adding 5 minutes to the timer
			if (timers[CURRENT_SELECTION] < now){ // if timer is expired
				timers[CURRENT_SELECTION] = now; // set it to current time
			}
			timers[CURRENT_SELECTION] = timers[CURRENT_SELECTION] + 1000 + snooze_time * 60 * 1000; // add snooze_time minutes
			/* alert_timeout = 0; */
			/* Heltec.display->normalDisplay(); // revert display just in case. it will invert if necessary */
			reset_alarm_state();
		}
		else if(input == 'w'){ // snooze the timer by adding 60 minutes to the timer
			if (timers[CURRENT_SELECTION] < now){ // if timer is expired
				timers[CURRENT_SELECTION] = now; // set it to current time
			}
			timers[CURRENT_SELECTION] = timers[CURRENT_SELECTION] + 1000 + big_snooze_time * 60 * 1000; // add snooze_time minutes
			/* alert_timeout = 0; */
			/* Heltec.display->normalDisplay(); // revert display just in case. it will invert if necessary */
			reset_alarm_state();
		}
		else if(input == 'x'){ // snooze the timer by adding 1 minutes to the timer
			if (timers[CURRENT_SELECTION] < now){ // if timer is expired
				timers[CURRENT_SELECTION] = now; // set it to current time
			}
			timers[CURRENT_SELECTION] = timers[CURRENT_SELECTION] + 1000 + small_snooze_time * 60 * 1000; // add snooze_time minutes
			/* alert_timeout = 0; */
			/* Heltec.display->normalDisplay(); // revert display just in case. it will invert if necessary */
			reset_alarm_state();
		}
		else if(input == 'd'){ // snooze the timer by adding 1 DAY to the timer
			if (timers[CURRENT_SELECTION] < now){ // if timer is expired
				timers[CURRENT_SELECTION] = now; // set it to current time
			}
			timers[CURRENT_SELECTION] = timers[CURRENT_SELECTION] + 1000 + (60*24) * 60 * 1000; // add a day
			/* alert_timeout = 0; */
			/* Heltec.display->normalDisplay(); // revert display just in case. it will invert if necessary */
			reset_alarm_state();
		}
		else if(input == 'u' && CURRENT_SELECTION != MAX_ENTRIES - 1){ // move entry down UNLESS LAST
			timercopyspace = timers[CURRENT_SELECTION+1]; // set holding slot to next selection down
			timers[CURRENT_SELECTION+1] = timers[CURRENT_SELECTION]; // fill next seledction down with current selection
			timers[CURRENT_SELECTION] = timercopyspace; // fill current selection with holding slot
			for (int m = 0; m < MSG_SIZE; m++){
				entrycopyspace[m] = timer_names[CURRENT_SELECTION + 1][m];
			}
			for (int m = 0; m < MSG_SIZE; m++){
				timer_names[CURRENT_SELECTION + 1][m] = timer_names[CURRENT_SELECTION][m];
				timer_names[CURRENT_SELECTION][m] = entrycopyspace[m];
			}
			CURRENT_SELECTION++;
		}
		else if(input == 'i' && CURRENT_SELECTION != 0){  // move entry up UNLESS FIRST
			timercopyspace = timers[CURRENT_SELECTION-1]; // set holding slot to next selection down
			timers[CURRENT_SELECTION-1] = timers[CURRENT_SELECTION]; // fill next seledction down with current selection
			timers[CURRENT_SELECTION] = timercopyspace; // fill current selection with holding slot
			for (int m = 0; m < MSG_SIZE; m++){
				entrycopyspace[m] = timer_names[CURRENT_SELECTION - 1][m];
			}
			for (int m = 0; m < MSG_SIZE; m++){
				timer_names[CURRENT_SELECTION - 1][m] = timer_names[CURRENT_SELECTION][m];
				timer_names[CURRENT_SELECTION][m] = entrycopyspace[m];
			}
			CURRENT_SELECTION--;
		}
		else if(input == 'v' ){
			LAST_SCREEN = CURRENT_SCREEN;
			reset_selection_and_change_screen(LARGE_TIMER_VIEW_SCREEN, CURRENT_SELECTION);
		}
	}
	else if (CURRENT_SCREEN == LARGE_TIMER_VIEW_SCREEN){ //               INPUT ON LARGE TIMER VIEW SCREEN
		if(input == 'h'){
			reset_selection_and_change_screen(LAST_SCREEN, CURRENT_SELECTION);
		}
		else if(input == 'j' ){
			/* CURRENT_SELECTION++; */
			if (CURRENT_SELECTION != MAX_ENTRIES - 1){CURRENT_SELECTION++;}
			draw_screen(CURRENT_SCREEN);
		}
		else if(input == 'k' ){
			/* CURRENT_SELECTION--; */
			if (CURRENT_SELECTION != 0){CURRENT_SELECTION--;}
			draw_screen(CURRENT_SCREEN);
		}
		else if(input == 'w'){ // snooze the timer by adding 5 minutes to the timer
			if (timers[CURRENT_SELECTION] < now){ // if timer is expired
				timers[CURRENT_SELECTION] = now; // set it to current time
			}
			timers[CURRENT_SELECTION] = timers[CURRENT_SELECTION] + 1000 + big_snooze_time * 60 * 1000; // add snooze_time minutes
			reset_alarm_state();
		}
		else if(input == 's'){ // snooze the timer by adding 5 minutes to the timer
			if (timers[CURRENT_SELECTION] < now){ // if timer is expired
				timers[CURRENT_SELECTION] = now; // set it to current time
			}
			timers[CURRENT_SELECTION] = timers[CURRENT_SELECTION] + 1000 + snooze_time * 60 * 1000; // add snooze_time minutes
			reset_alarm_state();
		}
		else if(input == 'x'){ // snooze the timer by adding 5 minutes to the timer
			if (timers[CURRENT_SELECTION] < now){ // if timer is expired
				timers[CURRENT_SELECTION] = now; // set it to current time
			}
			timers[CURRENT_SELECTION] = timers[CURRENT_SELECTION] + 1000 + small_snooze_time * 60 * 1000; // add snooze_time minutes
			reset_alarm_state();
		}
		else if(input == 'd'){ // snooze the timer by adding 1 DAY to the timer
			if (timers[CURRENT_SELECTION] < now){ // if timer is expired
				timers[CURRENT_SELECTION] = now; // set it to current time
			}
			timers[CURRENT_SELECTION] = timers[CURRENT_SELECTION] + 1000 + (60*24) * 60 * 1000; // add a day
			/* alert_timeout = 0; */
			/* Heltec.display->normalDisplay(); // revert display just in case. it will invert if necessary */
			reset_alarm_state();
		}
		else if(input == 'q'){ // blank out the current countdown timer
			timers[CURRENT_SELECTION] = 0;
			reset_alarm_state();
		}
	}
	else if (CURRENT_SCREEN == TIMER_SET_SCREEN){ //                 INPUT ON TIMER SET SCREEN
		if(input > 47 && input < 58){ // input is number
			int input_number = (input - 48);
			PICK_TIME[CURRENT_SELECTION] = input_number;
			if(CURRENT_SELECTION != 3 ){
				CURRENT_SELECTION++;
			}
		}
		if(input == 'h' ){
			if(CURRENT_SELECTION == 0 ){
				reset_selection_and_change_screen(TIMER_SCREEN, EDIT_SELECTION);
			}
			else{
				CURRENT_SELECTION--;
			}
		}
		if(input == 'j' ){  // REDUCE number
			if (PICK_TIME[CURRENT_SELECTION] <= 0){ // If trying to reduce from 0 entry
				if (CURRENT_SELECTION == 0){      // 1st digit (tens of hours)
					PICK_TIME[CURRENT_SELECTION] = 2;
				}
				else if (CURRENT_SELECTION == 1){ // 2nd digit (ones of hours)
					PICK_TIME[CURRENT_SELECTION] = 9;
				}
				else if (CURRENT_SELECTION == 2){ // 3rd digit (tens of minutes)
					PICK_TIME[CURRENT_SELECTION] = 5;
				}
				else if (CURRENT_SELECTION == 3){ // 4th digit (tens of minutes)
					PICK_TIME[CURRENT_SELECTION] = 9;
				}
			}
			else {
				PICK_TIME[CURRENT_SELECTION]--; // decrement entry
			}
		}
		if(input == 'k' ){ // INCREASE number
			if (PICK_TIME[CURRENT_SELECTION] >= 2 && CURRENT_SELECTION == 0){ // 1st digit (tens of hours)
				// if tens of hours spot is 2, reset to 0
				PICK_TIME[CURRENT_SELECTION] = 0;
			}
			else if (PICK_TIME[CURRENT_SELECTION] >= 9 && CURRENT_SELECTION == 1){ // 2nd digit (ones of hours)
				// if ones of hours spot is 9, reset to 0
				PICK_TIME[CURRENT_SELECTION] = 0;
			}
			else if (PICK_TIME[CURRENT_SELECTION] >= 5 && CURRENT_SELECTION == 2){ // 3rd digit (tens of minutes)
				// if tens of minutes spot is 5, reset to 0
				PICK_TIME[CURRENT_SELECTION] = 0;
			}
			else if (PICK_TIME[CURRENT_SELECTION] >= 9 && CURRENT_SELECTION == 3){ // 4th digit (ones of minutes)
				// if ones of minutes spot is 9, reset to 0
				PICK_TIME[CURRENT_SELECTION] = 0;
			}
			else { // otherwise increment normally
				PICK_TIME[CURRENT_SELECTION]++; // increment entry
			}
		}
		if(input == 'l' ){
			if(CURRENT_SELECTION == 3 ){ // enter the data
				int hours = int_concat(PICK_TIME[0], PICK_TIME[1]) ;
				int mins = int_concat(PICK_TIME[2], PICK_TIME[3]) ;
				timers[EDIT_SELECTION] = now
					+ (((long)hours * 60 * 60 * 1000)
						 +  ((long)mins * 60) * 1000);
				// maybe the last pick time is useful
				// PICK_TIME[0] = 0; PICK_TIME[1] = 0; PICK_TIME[2] = 0; PICK_TIME[3] = 0;
				/* reset_selection_and_change_screen(TIMER_SCREEN, EDIT_SELECTION); */
				string_entry_ptr = &timer_names[EDIT_SELECTION];
				LAST_SCREEN = TIMER_SCREEN;
				reset_selection_and_change_screen(STRING_EDIT_SCREEN, 0);
				/* reset_selection_and_change_screen(TIMER_SCREEN, EDIT_SELECTION); */
			}
			else{
				CURRENT_SELECTION++;
			}
		}
		if(input == 'c' ){
			// combine entries into a clock time
			int target_hours = int_concat(PICK_TIME[0], PICK_TIME[1]) ;
			int target_mins = int_concat(PICK_TIME[2], PICK_TIME[3]) ;
			if(    target_hours >= 0 // only accept valid clock times
						 && target_hours <= 23 
						 && target_mins  >= 0
						 && target_mins  <= 59
						 ){
				int current_hours = time_hours;
				int current_minutes = time_minutes;
				int minutes_in_future = 0;
				int hours_in_future = 0;
				for(minutes_in_future = 0; minutes_in_future < 60; minutes_in_future++){
					if(current_minutes == target_mins){
						break;
					}
					current_minutes++;
					if(current_minutes > 59){
						current_minutes = 0;
						current_hours++;
						if(current_hours > 23){
							current_hours = 0;
						}
					}
				}
				for(hours_in_future = 0; hours_in_future < 24; hours_in_future++){
					if(current_hours == target_hours){
						break;
					}
					current_hours++;
					if(current_hours > 23){
						current_hours = 0;
					}
				}
				// calculate how many milliseconds that is from now
				timers[EDIT_SELECTION] = now
					+ (((long)hours_in_future * 60 * 60 * 1000)
						 +  ((long)minutes_in_future * 60) * 1000);
				string_entry_ptr = &timer_names[EDIT_SELECTION];
				LAST_SCREEN = TIMER_SCREEN;
				reset_selection_and_change_screen(STRING_EDIT_SCREEN, 0);
			}
			else{ // else reset to a valid time
				PICK_TIME[0] = 2;
				PICK_TIME[1] = 3;
				PICK_TIME[2] = 5;
				PICK_TIME[3] = 9;
			}
		}
	}
	else if (CURRENT_SCREEN == STRING_EDIT_SCREEN){ //                     INPUT ON STRING EDIT SCREEN
		if(input == 8){ // backspace
			edit_position--;
			if (edit_position == 255){ // if backspace on zero position rolls over, reset it
				edit_position = 0;
			}
			/* notes[EDIT_SELECTION][edit_position] = 32; // set to space (this is not standard, but fine I think. */
			(*string_entry_ptr)[edit_position] = 32;
			/* (*string_entry_ptr)[edit_position] = 0; // nah backspace it to 0 */
			// this should be a zero, but if I backspace mid-string it will dump the rest.
			// maybe something like only make it a zero if the next spot is a zero?
			// "stringify00000000000000000000000" // "stringify"
			// "str0000fy00000000000000000000000" // "str"
		}
		else if(input == 13){ // return
			edit_position = 0;
			/* reset_selection_and_change_screen(LIST_SCREEN, EDIT_SELECTION); */
			reset_selection_and_change_screen(LAST_SCREEN, EDIT_SELECTION);
		}
		else if(input == 18){ // left arrow
			if ((edit_position != 0)){edit_position--;}
		}
		else if(input == 19){ // right arrow
			// Don't let me arrow past the message size OR over a null
			if ((edit_position != MSG_SIZE-1)) { // if not on the last entry
				if (notes[EDIT_SELECTION][edit_position] != 0){
					edit_position++;
				}
			}
		}
		else{
			if (input > 31){ // if it's a printable character
				(*string_entry_ptr)[edit_position] = input;
				// notes[EDIT_SELECTION][edit_position] = input;
				edit_position++;
				if (edit_position == MSG_SIZE-1){// no, don't get to the end position
					edit_position--;
				}
			}
		}
	}
	else if (CURRENT_SCREEN == BLOB_SCREEN){ //                     INPUT ON BLOB SCREEN
		if(input == 8){ // backspace
			if (blob_position != 0){ // if backspace on zero position, don't reduce
				blob_position--;
				/* blob_position = 0; */
			}
			/* notes[EDIT_SELECTION][blob_position] = 32; // set to space (this is not standard, but fine I think. */
			blob[blob_position] = 32;
			/* (blob)[blob_position] = 0; // nah backspace it to 0 */
			// this should be a zero, but if I backspace mid-string it will dump the rest.
			// maybe something like only make it a zero if the next spot is a zero?
			// "stringify00000000000000000000000" // "stringify"
			// "str0000fy00000000000000000000000" // "str"
		}
		else if(input == KEY_INSERT.letter){ // insert
			// move second to last position into last & ascend until current selection
			for (long e = (BLOB_SIZE-2); e > blob_position-1; e--){
				// copy the entry into next slot
				blob[e+1] = blob[e]; // set next entry to this entry
			}
			blob[blob_position] = 32;
		}
		else if(input == KEY_DELETE.letter){ // delete
			for (int e = blob_position; e < BLOB_SIZE-1; e++){
				blob[e] = blob[e+1]; // set next entry to this entry
			}
		}
		else if(input == 13){ // return
			blob[blob_position] = 13;
		}
		else if(input == 27){ // ESCAPE
			/* blob_position = 0; */
			/* reset_selection_and_change_screen(LIST_SCREEN, EDIT_SELECTION); */
			reset_selection_and_change_screen(LAST_SCREEN, EDIT_SELECTION);
		}
		else if(input == 18){ // left arrow
			if ((blob_position != 0)){blob_position--;}
		}
		else if(input == 19){ // right arrow
			// Don't let me arrow past the message size OR over a null
			if ((blob_position != BLOB_SIZE-1)) { // if not on the last entry
				if (blob[blob_position] != 0){
					blob_position++;
				}
			}
		}
		else{
			if (input > 31){ // if it's a printable character
				blob[blob_position] = input;
				// notes[EDIT_SELECTION][blob_position] = input;
				blob_position++;
				if (blob_position == BLOB_SIZE-1){// no, don't get to the end position
					blob_position--;
				}
			}
		}
	}
}
