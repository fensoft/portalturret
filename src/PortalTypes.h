enum class TurretMode {
  Automatic = 0,
  Manual = 1
};

enum class TurretState {
  Idle = 0,
  Activated = 1,
  Searching = 2,
  Engaging = 3,
  TargetLost = 4,
  PickedUp = 5,
  Shutdown = 6,
  ManualEngaging = 7,
  Rebooting = 8,
};

enum class ManualState {
  Idle,
  Opening,
  Closing,
  Firing,
};

#define WIFI_CRED_FILE "settings.txt"
#define STATIONARY_ANGLE 90