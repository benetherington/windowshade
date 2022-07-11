

# from adafruit_motor.stepper import (FORWARD, BACKWARD,
#                                     MICROSTEP, INTERLEAVE,
#                                     SINGLE, DOUBLE)
# from adafruit_motorkit import MotorKit
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


blink_assm = """
    set pindirs, 1          ; set pin 1 to output

loop:
    out x, 1
    jmp !x, loop
    set pins, 1 [31]
    nop         [31]
    nop         [31]
    set pins, 0
    jmp loop
"""

# assembled = adafruit_pioasm.assemble(blink_assm)
# sm = rp2pio.StateMachine( assembled, frequency=2000, first_out_pin=board.SDA, first_set_pin=board.SDA, auto_pull=True)

# sm = rp2pio.StateMachine(
#     assembled,
#     frequency=2000,
#     # init=adafruit_pioasm.assemble("set pindirs 1"),
    
#     # IO mapping
#     first_out_pin    =board.SDA,        # sm_config_set_out_pins(&c, pin_sda, 1);
#     first_set_pin    =board.SDA,        # sm_config_set_set_pins(&c, pin_sda, 1);
#     first_in_pin     =board.SDA,        # sm_config_set_in_pins(&c, pin_sda);
#     first_sideset_pin=board.SCL,        # sm_config_set_sideset_pins(&c, pin_scl);
#     jmp_pin          =board.SDA,        # sm_config_set_jmp_pin(&c, pin_sda);

#     out_shift_right  =False,            # sm_config_set_out_shift(&c, false, true, 16);
#     auto_pull        =True,
#     pull_threshold   =16,
    
#     in_shift_right   =False,            # sm_config_set_in_shift(&c, false, true, 8);
#     auto_push        =True,
#     push_threshold   =8,

#     # Try to avoid glitching the bus while connecting the IOs. Get things set
#     # up so that pin is driven down when PIO asserts OE low, and pulled up
#     # otherwise.
#     # gpio_pull_up(pin_scl);
#     # gpio_pull_up(pin_sda);
#     # uint32_t both_pins = (1u << pin_sda) | (1u << pin_scl);
#     # pio_sm_set_pins_with_mask(pio, sm, both_pins, both_pins);
#     # pio_sm_set_pindirs_with_mask(pio, sm, both_pins, both_pins);
#     # pio_gpio_init(pio, pin_sda);
#     # gpio_set_oeover(pin_sda, GPIO_OVERRIDE_INVERT);
#     # pio_gpio_init(pio, pin_scl);
#     # gpio_set_oeover(pin_scl, GPIO_OVERRIDE_INVERT);
#     # pio_sm_set_pins_with_mask(pio, sm, 0, both_pins);
# )

LIS_ADDRESS = 0x18

def write(data):
    bits_out = bytearray(LIS_ADDRESS)
    bits_out.append(data)
    sm.write(bits_out)
    
    bits_in = bytearray()
    sm.readinto(bits_in)
    print(bits_in)
    return bits_in

# boot, fifo_en | blank, blank | latch_int1, 4d4_int1, latch_int2, d4d_int2
# send reboot command
REG_CTRL5       = 0x24
ctrl5_reboot    = 0b10_00_0000
time.sleep(0.01)  # takes 5ms
# ODR, ODR, ODR, ODR | low_power | z_enable, y_enable, x_enable
# set data rate to 400, turn on low power mode
REG_CTRL1       = 0x20
ctrl1_cfg       = 0b0101_1_111
# click_int1, IA1_int1 IA2_int1, ZYXDA_int1, 321DA_int1
# fifo_watermark_int1, fifo_overrun_int1, blank
# turn on click interrupts on int1
REG_CTRL3       = 0x22
ctrl3_cfg       = 0b10000_00_0
# block_data, endian | full_scale, full_scale | high_res
# self_test, self_test | sim
# turn on BDU, set range to 4g, turn off high_res
REG_CTRL4       = 0x23
ctrl4_cfg       = 0b10_01_0_00_0
# boot, fifo_en | blank, blank | latch_int1, 4d4_int1, latch_int2, d4d_int2
# enable latching interrupt on int1
# REG_CTRL5     = set above
ctrl5_cfg       = 0b00_00_1000
# double_z, single_z, double_y, single_y, double_x, single_x
# Turn on singletap for all axes
REG_CLICKCFG    = 0x38
click_cfg       = 0b010101
# latch_click | click_threshold (7 bits)
REG_CLICKTHS    = 0x3A
threshold       = 0b0_0011110
# click time limit, (7 bits)
REG_TIMELIMIT   = 0x3B
time_limit      = 0b0001010
# click time latency (8 bits)
REG_TIMELATENCY = 0x3C
time_latency    = 0b00010100

# click time window (7 to 0)
REG_TIMEWINDOW  = 0x3D
time_window     = 0b11111111

# adc_en, temp_en | blank, blank, blank, blank, blank, blank
REG_TEMPCFG     = 0x1f
temp_config     = 0x00_000000




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

