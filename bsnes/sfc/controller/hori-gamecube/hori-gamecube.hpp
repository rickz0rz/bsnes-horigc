struct HoriGamecube : Controller {
  enum : uint {
    Up, Down, Left, Right, A, B, X, Y, LTrigger, LTriggerClick, RTrigger, RTriggerClick, Z,
    LeftAnalogX, LeftAnalogY, CAnalogX, CAnalogY, Start,
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

  static constexpr uint8 packet[8] = {
    0x00, 0x80, 0xFE, 0xFE, 0xFE, 0xFE, 0xFF, 0xFF
  };
};
