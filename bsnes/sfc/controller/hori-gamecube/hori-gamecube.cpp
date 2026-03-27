/*
-------------------------------------------------------------------------------
Hori / "LodgeNet" SNES Controller Protocol (Reverse Engineered)

This device emulates a hypothetical adapter that would sit between a
Hori-manufactured LodgeNet/GameCube-style controller and a SNES running a
factory/test ROM. The exact real hardware arrangement is still unknown, but the
current emulation does implement the core 64-bit response path well enough for
the controller to be usable.

This is NOT a standard SNES controller protocol.

Summary
-------
The SNES host bit-bangs a custom bidirectional serial protocol over the
controller port. Each transaction returns a 64-bit (8-byte) packet.

The protocol uses:
  - $4016 writes -> latch/phase control
  - $4016 reads  -> clock + data input (DATA1)
  - $4201 bit 6  -> auxiliary output (host -> controller)

Current emulation only drives DATA1. The auxiliary host->controller signaling
path is still only documented here and not decoded yet.

-------------------------------------------------------------------------------
Transaction Framing
-------------------------------------------------------------------------------
The host initiates a transfer with a latch preamble:

    0 -> 1 -> 0 -> 1

After this sequence, the controller must output a 64-bit response.

IMPORTANT:
- The host continues toggling 0/1 during the transfer.
- These toggles are NOT a reset signal.
- The controller must NOT restart shifting during the 64-bit stream.

Implementation rule:
- Detect the 0→1→0→1 sequence to start a packet
- Ignore latch transitions while streaming

-------------------------------------------------------------------------------
Bit Timing / Ordering
-------------------------------------------------------------------------------
- Total packet size: 64 bits (8 bytes)
- Bits are shifted LSB-first
- One bit is consumed per host read of $4016

-------------------------------------------------------------------------------
Decoded Packet Format
-------------------------------------------------------------------------------
Let B0..B7 be the logical decoded bytes:

  B0 = digital button byte
  B1 = status byte, with more digital buttons
  B2 = main stick X
  B3 = main stick Y
  B4 = C-stick X
  B5 = C-stick Y
  B6 = analog trigger L
  B7 = analog trigger R

-------------------------------------------------------------------------------
Wire Encoding (CRITICAL)
-------------------------------------------------------------------------------
Bytes are NOT sent directly as B0..B7.

Transmission rules:

- B0 and B1:
    sent directly (LSB-first)

- B2..B7:
    raw = bitreverse(B ^ 0xFF)
    then sent LSB-first

Equivalent:
    SNES sees: logical = bitreverse(raw) ^ 0xFF

-------------------------------------------------------------------------------
Neutral State
-------------------------------------------------------------------------------
The current implementation reports:

    B0 = 0x00
    B1 = 0x80

with both sticks centered and both analog triggers released, all encoded
through the same B2..B7 wire transform used for active input.

-------------------------------------------------------------------------------
Status Byte (B1)
-------------------------------------------------------------------------------
Current implementation:
    bit 7 = ready/valid
    bit 2 = digital L click
    bit 3 = digital R click
    bit 4 = Y
    bit 5 = X

Older reverse-engineering notes suggested additional mode/state values such as
0x90 and 0xA0, but those are not currently modeled.

-------------------------------------------------------------------------------
Analog Requirements
-------------------------------------------------------------------------------
Triggers:
    Released:  < 0x36
    Pressed:  >= 0xD2

Sticks:
    Center ≈ 0x80
    ROM performs calibration + region tests (3x3 grid)

-------------------------------------------------------------------------------
Digital Mapping Notes
-------------------------------------------------------------------------------
B0 currently maps as:
    bit 0 = A
    bit 1 = B
    bit 2 = Z
    bit 3 = Start
    bit 4 = Up
    bit 5 = Down
    bit 6 = Left
    bit 7 = Right

The LodgeNet front-panel buttons are currently exposed by aliasing them onto
otherwise impossible d-pad combinations:
    Order    = Up + Left + Right
    Reset    = Up + Down + Left + Right
    Menu     = Up + Down
    Hash     = Up + Down + Left
    Select   = Up + Down + Right
    Asterisk = Left + Right

-------------------------------------------------------------------------------
Host -> Controller Signaling (Advanced)
-------------------------------------------------------------------------------
The SNES also transmits a byte (via $4201 bit 6) during each byte read.

Observed command seeds:
    0x20, 0x60, 0xA0, 0xE0

This likely selects controller mode / report format.

Current emulation still does not decode this for normal operation.

-------------------------------------------------------------------------------
Implementation Notes
-------------------------------------------------------------------------------
- Do NOT use standard SNES latch-reset behavior
- Use a sync state machine to detect transaction start
- Stream exactly 64 bits per transaction
- After 64 bits, return idle high (1) until the next preamble
- B2–B7 must use inverted + bit-reversed encoding

-------------------------------------------------------------------------------
*/

HoriGamecube::HoriGamecube(uint port) : Controller(port) {
  latched = 0;
  counter = 0;
}

auto HoriGamecube::data() -> uint2 {
  // Not currently in an active 64-bit transfer:
  // idle high is the safest default.
  if(sync != SyncState::Streaming || counter >= 64) {
    return 1;
  }

  const uint byte = counter >> 3;
  const uint bit = counter & 7;  // LSB-first

  uint value = 0;

  if (byte == 0) {
    value = (b0 >> bit) & 1;
  } else if (byte == 1) {
    value = (b1 >> bit) & 1;
  } else if (byte == 2) {
    value = (controlStickX >> bit) & 1;
  } else if (byte == 3) {
    value = (controlStickY >> bit) & 1;
  } else if (byte == 4) {
    value = (cStickX >> bit) & 1;
  } else if (byte == 5) {
    value = (cStickY >> bit) & 1;
  } else if (byte == 6) {
    value = (lTrigger >> bit) & 1;
  } else if (byte == 7) {
    value = (rTrigger >> bit) & 1;
  } else {
    value = (packet[byte] >> bit) & 1;
  }

  counter++;

  // After 64 bits, stop streaming until the next valid preamble.
  if(counter >= 64) {
    sync = SyncState::Idle;
  }

  return value;
}

inline uint8 encode(uint8 b) {
  b ^= 0xFF;  // invert

  uint8 r = 0;
  for(int i = 0; i < 8; i++) {
    r = (r << 1) | (b & 1);
    b >>= 1;
  }
  return r;
}

auto HoriGamecube::latch(bool data) -> void {
  if(latched == data) return;
  latched = data;

  switch(sync) {
    case SyncState::Idle:
      if(data == 0) sync = SyncState::Saw0;
      break;

    case SyncState::Saw0:
      if(data == 1) sync = SyncState::Saw01;
      else          sync = SyncState::Saw0;
      break;

    case SyncState::Saw01:
      if(data == 0) sync = SyncState::Saw010;
      else          sync = SyncState::Idle;
      break;

    case SyncState::Saw010:
      if(data == 1) {
        sync = SyncState::Streaming;
        counter = 0;
      } else {
        sync = SyncState::Idle;
      }
      break;

    case SyncState::Streaming:
      // Ignore all latch toggles while streaming.
      // The ROM flips 0/1 per bit; that should not restart us.

      order = platform->inputPoll(port, ID::Device::HoriGamecube, Order) & 1;
      reset = platform->inputPoll(port, ID::Device::HoriGamecube, Reset) & 1;
      menu = platform->inputPoll(port, ID::Device::HoriGamecube, Menu) & 1;
      hash = platform->inputPoll(port, ID::Device::HoriGamecube, Hash) & 1;
      select = platform->inputPoll(port, ID::Device::HoriGamecube, Select) & 1;
      asterisk = platform->inputPoll(port, ID::Device::HoriGamecube, Asterisk) & 1;

      if (order) {
        up = 1; down = 0; left = 1; right = 1;
      } else if (reset) {
        up = 1; down = 1; left = 1; right = 1;
      } else if (menu) {
        up = 1; down = 1; left = 0; right = 0;
      } else if (hash) {
        up = 1; down = 1; left = 1; right = 0;
      } else if (select) {
        up = 1; down = 1; left = 0; right = 1;
      } else if (asterisk) {
        up = 0; down = 0; left = 1; right = 1;
      } else {
        up = platform->inputPoll(port, ID::Device::HoriGamecube, Up) & 1;
        down = platform->inputPoll(port, ID::Device::HoriGamecube, Down) & 1;
        left = platform->inputPoll(port, ID::Device::HoriGamecube, Left) & 1;
        right = platform->inputPoll(port, ID::Device::HoriGamecube, Right) & 1;
      }
      
      a = platform->inputPoll(port, ID::Device::HoriGamecube, A) & 1;
      b = platform->inputPoll(port, ID::Device::HoriGamecube, B) & 1;
      x = platform->inputPoll(port, ID::Device::HoriGamecube, X) & 1;
      y = platform->inputPoll(port, ID::Device::HoriGamecube, Y) & 1;
      z = platform->inputPoll(port, ID::Device::HoriGamecube, Z) & 1;
      start = platform->inputPoll(port, ID::Device::HoriGamecube, Start) & 1;

      l = platform->inputPoll(port, ID::Device::HoriGamecube, L) & 1;
      r = platform->inputPoll(port, ID::Device::HoriGamecube, R) & 1;

      b0 = a | b << 1 | z << 2 | start << 3 | up << 4 | down << 5 | left << 6 | right << 7;
      b1 = b1Ready | l << 2 | r << 3 | y << 4 | x << 5;

      controlStickX = encode(platform->inputPoll(port, ID::Device::HoriGamecube, ControlStickXAxis) + 128);
      controlStickY = encode(127 - (platform->inputPoll(port, ID::Device::HoriGamecube, ControlStickYAxis)));
      
      cStickX = encode(platform->inputPoll(port, ID::Device::HoriGamecube, CStickXAxis) + 128);
      cStickY = encode(127 - (platform->inputPoll(port, ID::Device::HoriGamecube, CStickYAxis)));

      lTrigger = encode(127 - (platform->inputPoll(port, ID::Device::HoriGamecube, LTrigger)));
      rTrigger = encode(127 - (platform->inputPoll(port, ID::Device::HoriGamecube, RTrigger)));

      break;
  }
}
