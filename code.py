# autocopy

from adafruit_motor.stepper import (FORWARD, BACKWARD,
                                    MICROSTEP, INTERLEAVE,
                                    SINGLE, DOUBLE)
from adafruit_motorkit import MotorKit
import adafruit_pioasm
import board
import neopixel
import rp2pio
import time

pixels = neopixel.NeoPixel(board.NEOPIXEL, 1)
pixels[0] = (0, 0, 0)


i2c_assm="""
.program i2c
.side_set 1 opt pindirs

; TX Encoding:
; | 15:10 | 9     | 8:1  | 0   |
; | Instr | Final | Data | NAK |
;
; If Instr has a value n > 0, then this FIFO word has no
; data payload, and the next n + 1 words will be executed as instructions.
; Otherwise, shift out the 8 data bits, followed by the ACK bit.
;
; The Instr mechanism allows stop/start/repstart sequences to be programmed
; by the processor, and then carried out by the state machine at defined points
; in the datastream.
;
; The "Final" field should be set for the final byte in a transfer.
; This tells the state machine to ignore a NAK: if this field is not
; set, then any NAK will cause the state machine to halt and interrupt.
;
; Autopull should be enabled, with a threshold of 16.
; Autopush should be enabled, with a threshold of 8.
; The TX FIFO should be accessed with halfword writes, to ensure
; the data is immediately available in the OSR.
;
; Pin mapping:
; - Input pin 0 is SDA, 1 is SCL (if clock stretching used)
; - Jump pin is SDA
; - Side-set pin 0 is SCL
; - Set pin 0 is SDA
; - OUT pin 0 is SDA
; - SCL must be SDA + 1 (for wait mapping)
;
; The OE outputs should be inverted in the system IO controls!
; (It's possible for the inversion to be done in this program,
; but costs 2 instructions: 1 for inversion, and one to cope
; with the side effect of the MOV on TX shift counter.)

do_nack:
    jmp y-- entry_point        ; Continue if NAK was expected
    irq wait 0 rel             ; Otherwise stop, ask for help

do_byte:
    set x, 7                   ; Loop 8 times
bitloop:
    out pindirs, 1         [7] ; Serialise write data (all-ones if reading)
    nop             side 1 [2] ; SCL rising edge
    wait 1 pin, 1          [4] ; Allow clock to be stretched
    in pins, 1             [7] ; Sample read data in middle of SCL pulse
    jmp x-- bitloop side 0 [7] ; SCL falling edge

    ; Handle ACK pulse
    out pindirs, 1         [7] ; On reads, we provide the ACK.
    nop             side 1 [7] ; SCL rising edge
    wait 1 pin, 1          [7] ; Allow clock to be stretched
    jmp pin do_nack side 0 [2] ; Test SDA for ACK/NAK, fall through if ACK

entry_point:
                               ; .wrap_target (no support from ada_pioasm)
    out x, 6                   ; Unpack Instr count
    out y, 1                   ; Unpack the NAK ignore bit
    jmp !x do_byte             ; Instr == 0, this is a data record.
    out null, 32               ; Instr > 0, remainder of this OSR is invalid
do_exec:
    out exec, 16               ; Execute one instruction per FIFO word
    jmp x-- do_exec            ; Repeat n + 1 times
jmp entry_point                ; .wrap
"""
assembled = adafruit_pioasm.assemble(i2c_assm)
sm = rp2pio.StateMachine(
    assembled,
    frequency=2000,
    # init=adafruit_pioasm.assemble("set pindirs 1"),
    
    # IO mapping
    first_out_pin    =board.SDA,        # sm_config_set_out_pins(&c, pin_sda, 1);
    first_set_pin    =board.SDA,        # sm_config_set_set_pins(&c, pin_sda, 1);
    first_in_pin     =board.SDA,        # sm_config_set_in_pins(&c, pin_sda);
    first_sideset_pin=board.SCL,        # sm_config_set_sideset_pins(&c, pin_scl);
    jmp_pin          =board.SDA,        # sm_config_set_jmp_pin(&c, pin_sda);

    out_shift_right  =False,            # sm_config_set_out_shift(&c, false, true, 16);
    auto_pull        =True,
    pull_threshold   =16,
    
    in_shift_right   =False,            # sm_config_set_in_shift(&c, false, true, 8);
    auto_push        =True,
    push_threshold   =8,

    # Try to avoid glitching the bus while connecting the IOs. Get things set
    # up so that pin is driven down when PIO asserts OE low, and pulled up
    # otherwise.
    # gpio_pull_up(pin_scl);
    # gpio_pull_up(pin_sda);
    # uint32_t both_pins = (1u << pin_sda) | (1u << pin_scl);
    # pio_sm_set_pins_with_mask(pio, sm, both_pins, both_pins);
    # pio_sm_set_pindirs_with_mask(pio, sm, both_pins, both_pins);
    # pio_gpio_init(pio, pin_sda);
    # gpio_set_oeover(pin_sda, GPIO_OVERRIDE_INVERT);
    # pio_gpio_init(pio, pin_scl);
    # gpio_set_oeover(pin_scl, GPIO_OVERRIDE_INVERT);
    # pio_sm_set_pins_with_mask(pio, sm, 0, both_pins);
)




# mk = MotorKit()
# for i in range(200):
    # mk.stepper2.onestep(style=INTERLEAVE)

"""
Single step FORWARD fails without a sleep, even a 0.001 sleep. Everything else works
fine. Microstep hums in intermediate positions, silent on %16==0, quiet on
%16==8, progressively louder between.

MICROSTEP -- 3200 steps per 360
INTERLEAVE -- 400 steps per 360
SINGLE -- 200 steps per 360
DOUBLE -- 200 steps per 360
"""

