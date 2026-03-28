/*
-------------------------------------------------------------------------------
Hori / "LodgeNet" SNES Controller Protocol

This is not a standard SNES controller. The host bit-bangs a custom 64-bit
transaction over the controller port, and the emulator currently only drives
DATA1.

Transaction rules used by this implementation:
- Detect the latch preamble `0 -> 1 -> 0 -> 1` to begin a transfer.
- Once streaming starts, ignore further latch transitions until 64 bits have
  been shifted.
- Shift one bit per host read, LSB-first.
- After 64 bits, return idle-high (`1`) until the next valid preamble.

Logical packet layout (`B0`..`B7`):
- `B0`: digital buttons (`A, B, Z, Start, Up, Down, Left, Right`)
- `B1`: status / extra buttons (`bit 7 = ready`, `bit 2 = L`, `bit 3 = R`,
  `bit 4 = Y`, `bit 5 = X`)
- `B2`: main stick X
- `B3`: main stick Y
- `B4`: C-stick X
- `B5`: C-stick Y
- `B6`: analog trigger L
- `B7`: analog trigger R

Wire encoding:
- `B0` and `B1` are sent directly, LSB-first.
- `B2`..`B7` are sent as `bitreverse(B ^ 0xff)`, still shifted LSB-first.

The SNES also appears to transmit auxiliary command data via `$4201 bit 6`, but
that host-to-controller path is not currently decoded here.
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
