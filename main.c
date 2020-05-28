//
// ATmega328 7-segment driver for 4 digits using segment multiplexing instead of
// digit multiplexing.  This allows for a simpler hardware design, with only 4 resistors
// and no transistor drivers because, at most, only 4 segments (one per digit) will
// be ON at any given time.
//
// Common anode and common cathode displays are supported, selectable at compile-time.
//
// A wire test function can be enabled at compile-time to verify correct wiring between
// the ATmega and the display.
//
// Seven output pins are used to drive the segments (no decimal point).
// Four output pins are used to drive the digit select (LED common) pins.
//
// Eight input pins are used to read an 8-bit value for display.
//
// One input pin is used to provide a LATCH function for the input data.  When the
// LATCH pin is HIGH then the 8-bit value on the DATA input lines is stored and
// displayed.  When the LATCH pin is LOW, the last stored data value is used.
//
// Two input pins are used for mode selection.  This allows the 8-bit data value to
// be displayed as HEX (00-ff), unsigned decimal (0-255) or signed decimal (-128-127).
// Changes to the mode selection are not latched by the LATCH pin.

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

// Uncomment only ONE of these to match the display type
#define COMMON_ANODE
//#define COMMON_CATHODE

// To make wiring easier for different displats, all ports and bits can be remapped
// here without no other code changes subject to the following restrictions:
//   MODE and digit selects must use a common port.
//   MODE and digit selects can be remapped into any order within the port.  Digit
//     selects do not need to be consecutive or in order.
//   Segment selects and LATCH must use a common port
//   Segment selects and LATCH can be remapped into any order within the port.
// Define port used for status and input.
#define SEGMENT_PORT PORTD
#define SEGMENT_INPUT PIND
#define SEGMENT_DIRECTION DDRD

#define CONTROL_PORT PORTC
#define CONTROL_INPUT PINC
#define CONTROL_DIRECTION DDRC

#define DATA_PORT PORTB
#define DATA_INPUT PINB
#define DATA_DIRECTION DDRB

// Control port bits
enum {
  CTL_MODE_MASK =   0x30,
  CTL_MODE_HEX =    0x00,
  CTL_MODE_UDEC =   0x10,
  CTL_MODE_SDEC =   0x20,
  CTL_MODE_BLANK =  0x30,
  CTL_DIGIT_MASK =  0x0f,   // digit select bits
};


//      A
//     ---
//  F | G | B
//     ---
//  E |   | C
//     ---
//      D

// LED segment bits
enum {
    SEG_LATCH = 0x80, // PD7
    SEG_MASK =  0x7f,
    SEG_G = 0x40, // PD6
    SEG_F = 0x20, // PD5
    SEG_E = 0x10, // PD4
    SEG_D = 0x08, // PD3
    SEG_C = 0x04, // PD2
    SEG_B = 0x02, // PD1
    SEG_A = 0x01  // PD0
};

// Additional characters to display beyond 0-9 and A-F.  Used to index char_map.
enum {
  NEG = 16,
  BLANK = 17
};

// Character definitions
enum {
    CHAR_0 = SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F,
    CHAR_1 = SEG_B|SEG_C,
    CHAR_2 = SEG_A|SEG_B|SEG_D|SEG_E|SEG_G,
    CHAR_3 = SEG_A|SEG_B|SEG_C|SEG_D|SEG_G,
    CHAR_4 = SEG_B|SEG_C|SEG_F|SEG_G,
    CHAR_5 = SEG_A|SEG_C|SEG_D|SEG_F|SEG_G,
    CHAR_6 = SEG_A|SEG_C|SEG_D|SEG_E|SEG_F|SEG_G,
    CHAR_7 = SEG_A|SEG_B|SEG_C,
    CHAR_8 = SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F|SEG_G,
    CHAR_9 = SEG_A|SEG_B|SEG_C|SEG_F|SEG_G,
    CHAR_A = SEG_A|SEG_B|SEG_C|SEG_E|SEG_F|SEG_G,
    CHAR_B = SEG_C|SEG_D|SEG_E|SEG_F|SEG_G,
    CHAR_C = SEG_A|SEG_D|SEG_E|SEG_F,
    CHAR_D = SEG_B|SEG_C|SEG_D|SEG_E|SEG_G,
    CHAR_E = SEG_A|SEG_D|SEG_E|SEG_F|SEG_G,
    CHAR_F = SEG_A|SEG_E|SEG_F|SEG_G,
    CHAR_NEG = SEG_G,
    CHAR_BLANK = 0
};

// Mapping of indexes to characters
const uint8_t char_map[] = {
    CHAR_0, CHAR_1, CHAR_2, CHAR_3, CHAR_4, CHAR_5, CHAR_6, CHAR_7,
    CHAR_8, CHAR_9, CHAR_A, CHAR_B, CHAR_C, CHAR_D, CHAR_E, CHAR_F,
    CHAR_NEG, CHAR_BLANK
};

void ioinit (void); // initiate IO on the AVR
void delay_and_check_for_data(void);
void build_maps(uint8_t data, uint8_t mode);
void write_segment(uint8_t seg);
void write_digit_selects(uint8_t digit);
void wire_test(void);
void delay_ms (uint16_t x);


// Array that indicates which segments are on for each digit.
// The leftmost digit is 0 and the rightmost is 3.
uint8_t segments_by_digit[4];

// Array that indicates which digits are enabled for each segment.  Each entry has
// bits that are on for the digits that enabled when a segment is lit.  This
// array answers questions of the form, "which digits have their A segment lit?"
uint8_t digits_by_segment[8];


int main (void)
{
    // Initiate IO ports and peripheral devices.
    ioinit();

    // Uncomment this to verify the wiring between the ATmega328 and the display.
    // It will loop forever, slowly cycling the segments A-G on each digit in turn.
    //wire_test();  // Never returns!!!

    // Start with display off
    build_maps(0, CTL_MODE_BLANK);

    while (1) {
        write_segment(0);
        write_digit_selects(0);

        // Loop over the segments, activating one at a time and set the digit selects to
        // light that segment on all appropriate digits of the display.  At any given
        // time, a single segment is shown on any or all displays.  This means that that
        // the chip is only ever driving 4 segment max, one per digit.
        for (int ix = 0; (ix < 8); ix++) {
            uint8_t seg_bit = 1 << ix;
            if ((seg_bit & SEG_MASK) == 0) {
                // This pin on the port is not used to drive a segment.
                continue;
            }
            write_segment(seg_bit);
            write_digit_selects(digits_by_segment[ix]);
            delay_and_check_for_data();
        }
    }
}

void ioinit (void)
{
    // ### Initiate I/O

    // Data Direction Registers
    // Bit set to 1 means it works as an output
    // Bit set to 0 means it is an input
    SEGMENT_DIRECTION = SEG_MASK;       // LED segments as output, LATCH as input
    CONTROL_DIRECTION = CTL_DIGIT_MASK; // Digit controls as output, MODE as input
    DATA_DIRECTION = 0x00;              // Input 8-bit value

    // Set all output ports OFF and no pull up resistors on any inputs.
    DATA_PORT = 0x00;
    SEGMENT_PORT = 0x00;
    CONTROL_PORT = 0x00;
}

// Check for changes on the data and mode pins.  If no change is detected then the check
// loops multiple time to provide a short time delay to allow a single segment to remain
// lit.  If a change is detected, the maps are rebuilt and this code exits to allow the
// new data to be displayed.  By combining the segment time delay and pin change detection
// functions, the signal changes can acted on quickly without using ISRs.
void delay_and_check_for_data(void) {
    static uint8_t diplay_data = 0;
    static uint8_t display_mode = CTL_MODE_BLANK;
    static const int SEGMENT_DELAY = 148; // 2 ms delay to display a segment

    // Loop for  about 2ms or until the data or mode changes.
    for (int loop = SEGMENT_DELAY; (loop); loop--) {
        // The LATCH bit is active low, so only use new data values when the LATCH
        // bit is HIGH.  The data value won't change (latched) when the bit is LOW.
        // NOTE that the data value is read before the latch value to avoid using
        // data on a transition to the latched state at the instant between the
        // latch port read and the data port read.
        uint8_t d = DATA_INPUT;
        uint8_t data_valid = (SEGMENT_INPUT & SEG_LATCH);
        uint8_t m = CONTROL_INPUT & CTL_MODE_MASK;
        if ((data_valid && (d != display_data)) || (m != display_mode)) {
            // If the data value or the display mode changed then update the maps to
            // change the displayed value.
            display_data = d;
            display_mode = m;
            build_maps(display_data, display_mode);
            break;
        }
    }
}

// Built the segments_by_digit and digits_by_segment arrays based on the data and mode.
void build_maps(uint8_t data, uint8_t mode) {
    int sd; // Large enough to hold abs(-128) in SDEC mode.

    // Build the array that shows which segments are on for each digit.
    switch(mode) {
    case CTL_MODE_HEX:
        // For 8 bit hex, values are "  00" to "  FF"
        segments_by_digit[0] = char_map[BLANK];
        segments_by_digit[1] = char_map[BLANK];
        segments_by_digit[2] = char_map[data >> 4];
        segments_by_digit[3] = char_map[data & 0x0f];
        break;
    case CTL_MODE_SDEC:
    case CTL_MODE_UDEC:
        sd = (mode == CTL_MODE_SDEC) ? (int8_t) data : data;
        // For 8-bit signed, values are "-128" to " 127"
        // For 8-bit unsigned, values are " 000" to " 255"
        if (sd < 0) {
            segments_by_digit[0] = char_map[NEG];
            sd = -sd;
        } else {
            segments_by_digit[0] = char_map[BLANK];
        }
        segments_by_digit[3] = char_map[sd % 10];
        sd = sd / 10;
        segments_by_digit[2] = char_map[sd % 10];
        sd = sd / 10;
        segments_by_digit[1] = char_map[sd % 10];
        break;
    case CTL_MODE_BLANK:
        // All off
        segments_by_digit[0] = char_map[BLANK];
        segments_by_digit[1] = char_map[BLANK];
        segments_by_digit[2] = char_map[BLANK];
        segments_by_digit[3] = char_map[BLANK];
        break;
    }

    // Build the array that shows which digits are enabled for each segment.
    for (uint8_t seg = 0; (seg < 8); seg++) {
        uint8_t seg_bit = 1 << seg;
        digits_by_segment[seg] = 0;
        for (uint8_t dig = 0; (dig < 4); dig++) {
            uint8_t dig_bit = 1 << dig;
            if (segments_by_digit[dig] & seg_bit) {
                digits_by_segment[seg] |= dig_bit;
            }
        }
    }
}

void write_segment(uint8_t seg) {
#ifdef COMMON_ANODE
    // For common anode, segments are active LOW and digit selects are active HIGH
    seg = ~seg;
#endif
    seg &= SEG_MASK;
    SEGMENT_PORT = seg;
}

void write_digit_selects(uint8_t digit) {
#ifdef COMMON_CATHODE
    // For common cathode, segments are active HIGH and digit selects are active LOW
    digit = ~digit;
#endif
    digit &= CTL_DIGIT_MASK;
    CONTROL_PORT = digit;
}

void wire_test(void) {
    const uint16_t TEST_DELAY = 1000;

    write_segment(0);
    write_digit_selects(0);
    while (1) {
        for (int digit_bit = 1; (digit_bit); digit_bit <<= 1) {
            if ((digit_bit & CTL_DIGIT_MASK) == 0) {
                continue;
            }
            write_digit_selects(digit_bit);
            write_segment(SEG_A);
            delay_ms(TEST_DELAY);
            write_segment(SEG_B);
            delay_ms(TEST_DELAY);
            write_segment(SEG_C);
            delay_ms(TEST_DELAY);
            write_segment(SEG_D);
            delay_ms(TEST_DELAY);
            write_segment(SEG_E);
            delay_ms(TEST_DELAY);
            write_segment(SEG_F);
            delay_ms(TEST_DELAY);
            write_segment(SEG_G);
            delay_ms(TEST_DELAY);
        }
    }
}

// Spin loop to provide a delay
void delay_ms(uint16_t x)
{
  uint8_t y, z;
  for ( ; x > 0 ; x--){
    for ( y = 0 ; y < 90 ; y++){
      for ( z = 0 ; z < 6 ; z++){
        asm volatile ("nop");
      }
    }
  }
}
