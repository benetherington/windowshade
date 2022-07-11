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

i2c_assembled = adafruit_pioasm.assemble(i2c_assm)

sm = rp2pio.StateMachine(i2c_assembled, frequency=2000,
    # IO mapping
    first_out_pin    =board.SDA,        # sm_config_set_out_pins(&c, pin_sda, 1);
    first_set_pin    =board.SDA,        # sm_config_set_set_pins(&c, pin_sda, 1);
    first_in_pin     =board.SDA,        # sm_config_set_in_pins(&c, pin_sda);
    jmp_pin          =board.SDA,        # sm_config_set_jmp_pin(&c, pin_sda);
    first_sideset_pin=board.SCL,        # sm_config_set_sideset_pins(&c, pin_scl);

    # FIFO management
    auto_push        =True,
    push_threshold   =8,
    out_shift_right  =False,            # sm_config_set_out_shift(&c, false, true, 16);
    
    auto_pull        =True,
    pull_threshold   =16,
    in_shift_right   =False,            # sm_config_set_in_shift(&c, false, true, 8);
)

# sm.write(array("I", [0b1111_0000_1111_0000_1100_1100_1010_1010]))
# sm.readinto(array("B", [0]*32))



# Assemble a table of instructions which software can select from, and pass
# into the FIFO, to issue START/STOP/RSTART. This isn't intended to be run as
# a complete program.

I2C_SC0_SD0 = 0
I2C_SC0_SD1 = 1
I2C_SC1_SD0 = 2
I2C_SC1_SD1 = 3
set_scl_sda_program_instructions = [
    adafruit_pioasm.assemble(""".program set_scl_sda
                                             .side_set 1 opt
                                                 set pindirs, 0 side 0 [7] ; SCL = 0, SDA = 0"""),
    adafruit_pioasm.assemble(""".program set_scl_sda
                                             .side_set 1 opt
                                                 set pindirs, 1 side 0 [7] ; SCL = 0, SDA = 1"""),
    adafruit_pioasm.assemble(""".program set_scl_sda
                                             .side_set 1 opt
                                                 set pindirs, 0 side 1 [7] ; SCL = 1, SDA = 0"""),
    adafruit_pioasm.assemble(""".program set_scl_sda
                                             .side_set 1 opt
                                                 set pindirs, 1 side 1 [7] ; SCL = 1, SDA = 1"""),
]


# TX Encoding:
# | 15:10 | 9     | 8:1  | 0   |
# | Instr | Final | Data | NAK |
PIO_I2C_ICOUNT_LSB = 10
PIO_I2C_FINAL_LSB  = 9
PIO_I2C_DATA_LSB   = 1
PIO_I2C_NAK_LSB    = 0

# BUILT
def pio_sm_drain_tx_fifo(pio, sm):
    sm.clear_rx_fifo()
def pio_sm_is_rx_fifo_empty(pio, sm):
    return sm.in_waiting

# ADAPTED
def pio_i2c_check_error(pio, sm):
    return pio_interrupt_get(pio, sm)

def pio_i2c_resume_after_error(pio, sm):
    pio_sm_drain_tx_fifo(pio, sm)
    sm.run( # (pio->sm[sm].execctrl &
        PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS >> PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB
    )
    pio_interrupt_clear(pio, sm)

def pio_i2c_rx_enable(pio, sm, enable):
    if enable:
        sm.auto_push = True
    else:
        sm.auto_push = False

def pio_i2c_put16(pio, sm, data):
    while pio_sm_is_tx_fifo_full(pio, sm):
        sm.write(data)
        # *(io_rw_16 *)&pio->txf[sm] = data;


# If I2C is ok, block and push data. Otherwise fall straight through.
def pio_i2c_put_or_err(pio, sm, data)
    while pio_sm_is_tx_fifo_full(pio, sm):
        if pio_i2c_check_error(pio, sm):
            return
    if pio_i2c_check_error(pio, sm):
        return
    # *(io_rw_16 *)&pio->txf[sm] = data

def pio_i2c_get(pio, sm):
    return sm #pio_sm_get(pio, sm)

def pio_i2c_start(pio, sm):
    pio_i2c_put_or_err(pio, sm, 0b1 << PIO_I2C_ICOUNT_LSB); # Escape code for 2 instruction sequence
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC1_SD0])    # We are already in idle state, just pull SDA low
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC0_SD0])    # Also pull clock low so we can present data

def pio_i2c_stop(pio, sm):
    pio_i2c_put_or_err(pio, sm, 0b10 << PIO_I2C_ICOUNT_LSB);
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC0_SD0])    # SDA is unknown; pull it down
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC1_SD0])    # Release clock
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC1_SD1])    # Release SDA to return to idle state

def pio_i2c_repstart(pio, sm):
    pio_i2c_put_or_err(pio, sm, 0b10 << PIO_I2C_ICOUNT_LSB);
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC0_SD1])
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC1_SD1])
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC1_SD0])
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC0_SD0])

def pio_i2c_wait_idle(pio, sm):
    # Finished when TX runs dry or SM hits an IRQ
    pio->fdebug = 0b1 << (PIO_FDEBUG_TXSTALL_LSB + sm)
    while not (   "pio->fdebug" & 0b1 << (PIO_FDEBUG_TXSTALL_LSB + sm)
                or pio_i2c_check_error(pio, sm)):
        tight_loop_contents()

def pio_i2c_write_blocking(pio, sm, addr, txbuf, length):
    err = 0
    pio_i2c_start(pio, sm)
    pio_i2c_rx_enable(pio, sm, False)
    pio_i2c_put16(pio, sm, (addr << 2) | 0b1)
    while length and not pio_i2c_check_error(pio, sm):
        if not pio_sm_is_tx_fifo_full(pio, sm):
            length -= 1
            txbuf += 1
            pio_i2c_put_or_err(pio, sm,
                  txbuf << PIO_I2C_DATA_LSB
                | ((len == 0) << PIO_I2C_FINAL_LSB)
                | 0b1
            )
    pio_i2c_stop(pio, sm)
    pio_i2c_wait_idle(pio, sm)
    if pio_i2c_check_error(pio, sm):
        err = -1
        pio_i2c_resume_after_error(pio, sm)
        pio_i2c_stop(pio, sm)
    return err

def pio_i2c_read_blocking(pio, sm, addr, rxbuf, length):
    err = 0
    pio_i2c_start(pio, sm)
    pio_i2c_rx_enable(pio, sm, True)
    while not pio_sm_is_rx_fifo_empty(pio, sm):
        pio_i2c_get(pio, sm)
    pio_i2c_put16(pio, sm, (addr << 2) | 0b11)
    tx_remain = length # Need to stuff 0xff bytes in to get clocks

    first = True

    while (tx_remain or length) and not pio_i2c_check_error(pio, sm):
        if tx_remain and not pio_sm_is_tx_fifo_full(pio, sm):
            tx_remain -= 1
            pio_i2c_put16(pio, sm,
                  0xff << 1
                | 0 if tx_remain else
                      (0b1 << PIO_I2C_FINAL_LSB)
                    | (0b1 << PIO_I2C_NAK_LSB)
            )
        if not pio_sm_is_rx_fifo_empty(pio, sm):
            if first:
                # Ignore returned address byte
                pio_i2c_get(pio, sm)
                first = False
            else:
                length -= 1
                rxbuf += 1
                pio_i2c_get(pio, sm) # increment rxbuf pointer and set that to _get
    pio_i2c_stop(pio, sm)
    pio_i2c_wait_idle(pio, sm)
    if pio_i2c_check_error(pio, sm):
        err = -1
        pio_i2c_resume_after_error(pio, sm)
        pio_i2c_stop(pio, sm)
    return err


