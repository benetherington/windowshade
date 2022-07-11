#autocopy

import adafruit_pioasm
import board
import rp2pio
from array import array

# LIS_ADDRESS = 0x18

# def write(sm, data):
#     bits_out = bytearray(LIS_ADDRESS)
#     bits_out.append(data)
#     sm.write(bits_out)
    
#     bits_in = bytearray()
#     sm.readinto(bits_in)
#     print(bits_in)
#     return bits_in








assembled = adafruit_pioasm.assemble("""


""")

sm = rp2pio.StateMachine(assembled, frequency=2000,
    # IO mapping
    first_out_pin           =board.SDA,        # sm_config_set_out_pins(&c, pin_sda, 1);
    initial_out_pin_state   =0,
    first_set_pin           =board.SDA,        # sm_config_set_set_pins(&c, pin_sda, 1);
    initial_set_pin_state   =0,
    first_in_pin            =board.SDA,        # sm_config_set_in_pins(&c, pin_sda);
    in_pin_count            =2,
    jmp_pin                 =board.SDA,        # sm_config_set_jmp_pin(&c, pin_sda);
    first_sideset_pin       =board.SCL,        # sm_config_set_sideset_pins(&c, pin_scl);

    # FIFO management
    auto_push        =True,
    push_threshold   =8,
    out_shift_right  =False,            # sm_config_set_out_shift(&c, false, true, 16);
    
    auto_pull        =True,
    pull_threshold   =16,
    in_shift_right   =False,            # sm_config_set_in_shift(&c, false, true, 8);
)

lis_address = 0x18
who_am_i = 0x0F
