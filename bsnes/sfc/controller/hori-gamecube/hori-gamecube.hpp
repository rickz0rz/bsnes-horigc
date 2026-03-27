struct HoriGamecube : Controller {
  enum : uint {
    Up, Down, Left, Right, A, B, X, Y, L, LTrigger, R, RTrigger, Z,
    LeftAnalogXAxis, LeftAnalogYAxis, CAnalogXAxis, CAnalogYAxis, Start,
  };

  HoriGamecube(uint port);

  auto data() -> uint2;
  auto latch(bool data) -> void;

private:
enum class SyncState : uint {
    Idle,        // waiting
    Saw0,        // saw first 0
    Saw01,       // saw 0 -> 1
    Saw010,      // saw 0 -> 1 -> 0
    Streaming    // saw 0 -> 1 -> 0 -> 1, now emit 64 bits
  };

  bool latched = false;
  SyncState sync = SyncState::Idle;
  uint counter = 64;  // not streaming initially

  bool up, down, left, right, a, b, x, y, z, l, r, start;
  uint leftAnalogX, leftAnalogY, cAnalogX, cAnalogY, lTrigger, rTrigger;
  
  // rendered button bytes.
  uint b0, b1;

  static constexpr uint8 packet[8] = {
    0x00,
    0x80,
    0xFE, // Left analog stick, X
    0xFF, // Left analog stick, Y
    0xFE, // C analog stick, X
    0xFE, // C analog stick, Y
    0xFF, // Left trigger analog
    0xFF  // Right trigger analog
  };
};
