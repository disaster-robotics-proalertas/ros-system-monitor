byte ERROR=0
byte BOOT=1
byte SERVICE=2
byte RECORDING=3

Header header
string name         # State name
string description  # State description (e.g., "Waiting for GPS fix")
byte id             # Current vehicle status id (see definitions above)