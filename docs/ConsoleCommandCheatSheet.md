# MCB Console Command Cheat Sheet

Quick reference for the commands the MCB accepts on its USB (debug) serial
console. These are the same commands the PIB sends over the DIB port. Numbers
are the message id. Source of truth: `MCBComm.h` (enum `MCBMessages_t`),
`src/InternalSerialDriverMCB.cpp` (parsing), `src/MCB.cpp` and `src/States.cpp`
(behaviour). Values are for the **RACHuTS** build unless noted. Last updated
2026-10-24.

---

## Sending a command

USB console at **115200 baud**. Each command is one line:

```
#<id>;                     no parameters      e.g.  #9;
#<id>,<p1>,<p2>,...;       with parameters    e.g.  #24,10,100;
```

- Send `m` on its own to print the built-in command menu.
- The whole command must arrive within **100 ms**, so send a complete line
  (as a serial monitor does on Enter); don't type it one character at a time.
- No checksum is needed. The MCB doesn't check it.
- Parameters are decimal floats (`10`, `2.5`, `-15`).

## Where replies go

Most replies (ACKs, `MOTION_FINISHED`, temperature/voltage/current TM,
EEPROM dump) go to the **DIB/PIB port, not the USB console**, even when the
command came from USB. On the console you see:

- debug prints such as `Reeling in: 10.00` and `Reel motion complete`.
- every MCB error, echoed as `TX ERR: <message>`. The same text goes to the
  PIB, which reports it to the ground as an `MCBSTRING` TM and enters its
  error state.
- the immediate ACK for commands 7, 20, 21 and 22, and the
  "Bad parameters ... ignored" error for a malformed command.

## MCB states

Motion commands are only accepted in **READY** or **NOMINAL** (low power).
In any other state they're rejected with
`<command> denied, reel ops ongoing (state <STATE>)`.

States: `READY`, `NOMINAL`, `REEL_OUT`, `REEL_IN`, `DOCK`, `IN_NO_LW`,
`HOME_LW`, `CENTER_LW`.

Every motion (including home/center LW) first checks that the **15 V rail is
above 14.0 V**. If it isn't, it fails with
`Voltage too low to <op>: 15V rail x.xx V < 14.0 V`.

---

## State / mode (7–9)

| Id | Name | Description | Example |
|----|------|-------------|---------|
| 7 | CANCEL_MOTION | Stop any motion, go to READY (immediate ACK) | `#7;` |
| 8 | GO_LOW_POWER | Go to NOMINAL (low power, controllers off) | `#8;` |
| 9 | GO_READY | Go to READY | `#9;` |

## Reel motion (24–27, 20)

| Id | Name | Description | Params | Example |
|----|------|-------------|--------|---------|
| 24 | REEL_OUT | Reel out (deploy) | revs, speed (rpm) | `#24,10,100;` |
| 25 | REEL_IN | Reel in with level wind camming | revs, speed (rpm) | `#25,10,100;` |
| 26 | DOCK | Dock reel-in. Level wind must be homed or centered first | revs, speed (rpm) | `#26,2,80;` |
| 27 | IN_NO_LW | Reel in without the level wind | revs, speed (rpm) | `#27,2,50;` |
| 20 | FULL_RETRACT | Stop motion, then reel in 99.5 % of the stored position at 250 rpm (immediate ACK) | — | `#20;` |

- Accepted ranges (RACHuTS): revs **0–8000**, speed **0–400 rpm**. Zero is
  rejected, so both must be greater than 0.
- Acceleration isn't a parameter here. Each motion uses the stored value set
  by commands 28–30.
- The speed you send is also saved to EEPROM as that motion's velocity.
- An out-of-range value is rejected when the motion starts, e.g.
  `Error commanding reel in for dock: acc 80 not in (0, 8.20] rot/s^2`.

## Acceleration (28–30), stored in EEPROM

| Id | Name | Used by | Params | Example |
|----|------|---------|--------|---------|
| 28 | OUT_ACC | Reel out (24) | accel (rot/s²) | `#28,4;` |
| 29 | IN_ACC | Reel in (25, 27, 20) | accel (rot/s²) | `#29,4;` |
| 30 | DOCK_ACC | Dock (26) | accel (rot/s²) | `#30,4;` |

RACHuTS default **4.0**, allowed range **0–8.2** rot/s² (must be greater
than 0). The value is stored without a range check. A bad value only shows up
as an error at the next motion that uses it, and it persists across reboots.

> RATS and FLOATS use different units and limits (default 80, max 82 rot/s²,
> max 100 rpm). See `src/Reel.h`. An EEPROM written by one of those builds
> fails every RACHuTS motion until 28–30 are resent.

## Level wind and reel position (10–12)

| Id | Name | Description | Example |
|----|------|-------------|---------|
| 10 | HOME_LW | Home the level wind (runs 30 s, then READY) | `#10;` |
| 11 | CENTER_LW | Home, then center the level wind (needed before DOCK) | `#11;` |
| 12 | ZERO_REEL | Set the reel position to 0. On RACHuTS it's recorded immediately but only written to the controller at the next motion, so the brake isn't released while docked | `#12;` |

## Controllers and brake (16–19)

| Id | Name | Description | Example |
|----|------|-------------|---------|
| 16 | BRAKE_ON | Engage reel brake | `#16;` |
| 17 | BRAKE_OFF | Release reel brake (reel controller must already be on) | `#17;` |
| 18 | CONTROLLERS_ON | Power reel and level wind controllers (READY only) | `#18;` |
| 19 | CONTROLLERS_OFF | Power both controllers off | `#19;` |

## Limits (31–33, 21–22), stored in EEPROM

| Id | Name | Description | Params | Example |
|----|------|-------------|--------|---------|
| 31 | TEMP_LIMITS | Motor/controller temperature limits (°C) | mtr1_hi, mtr1_lo, mtr2_hi, mtr2_lo, mc1_hi, mc1_lo | `#31,80,-15,60,-15,80,-40;` |
| 32 | TORQUE_LIMITS | Reel torque limits (drive units) | reel_hi, reel_lo | `#32,500,-500;` |
| 33 | CURR_LIMITS | Reel motor (MTR1) current limits (A) | hi, lo | `#33,13.75,-10;` |
| 21 | IGNORE_LIMITS | Disable limit enforcement (immediate ACK) | — | `#21;` |
| 22 | USE_LIMITS | Re-enable limit enforcement (immediate ACK) | — | `#22;` |

The examples show the defaults. Limits only stop motion outside READY/NOMINAL.
A trip sends e.g. `MCB Curr Limit: MTR1 14.02 A > 13.75 A` and switches to
READY. The 21/22 setting is saved in EEPROM, so it survives a reboot.

> RATS build: every boot starts with limits ignored and the reel torque limit
> off, whatever EEPROM holds, so 21/22 and 32 only last until the next power
> cycle. See README.md.

## Status requests (13–15, 23)

Replies go to the **DIB/PIB port** and don't appear on the USB console.

| Id | Name | Returns | Example |
|----|------|---------|---------|
| 13 | GET_TEMPERATURES | Motor/controller temperatures | `#13;` |
| 14 | GET_VOLTAGES | 3V3, 15V, 20V, spool rail voltages | `#14;` |
| 15 | GET_CURRENTS | Brake, MC, MTR1, MTR2, instrument currents | `#15;` |
| 23 | GET_EEPROM | Binary dump of the stored config (velocities, accelerations, limits) | `#23;` |

---

## Typical bench sequence

```
#9;              ready
#28,4;           set out / in / dock accelerations
#29,4;
#30,4;
#11;             home + center level wind (wait for it to finish)
#24,5,100;       reel out 5 rev at 100 rpm
#25,5,100;       reel back in
#26,1,80;        dock
#8;              low power
```

To stop anything at once: `#7;`
