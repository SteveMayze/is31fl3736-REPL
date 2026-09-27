# is31fl3736-REPL

A REPL over the is31fl3736 chip LED driver for controlling LED matrices in real-time.


# Serial commands:
* help
* mode pwm                              -> global PWM mode (B_EN=0)
* mode abm                              -> global Auto Breath mode (B_EN=1)
* load pwm v0,v1,v2, v3,v4,v5, ...       -> flat byte list, 3 values = one RGB dot-triplet,
                                          written to PWM regs starting at dot 0;
                                          non-zero value also turns that dot's On/Off bit on
* load abm m0,m1,m2, ...                 -> flat list of ABM modes (0-3), assigned to dots
                                          starting at dot 0, in order
* assign abm <n> <dot...>               -> same assignment, but by explicit dot index
                                          (n = 0 for PWM control, 1-3 for ABM-1..3)
* define abm <n> <T1> <T2> <T3> <T4>    -> program ABM-n timing (raw register codes, not
                                          seconds - see Table 15/16 of the datasheet) and
                                          commit per Figure 16 (clear/set B_EN, update 0Eh)
* gcc <0-255>                           -> Global Current Control (PG3, 01h)
* reset                                 -> trigger IC reset (read PG3, 11h) and re-init
* dump                                  -> print current shadow state

