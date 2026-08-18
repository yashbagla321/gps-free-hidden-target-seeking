# Native relay range-bearing packet, expressed in the relay's own frame B.
# stamp is the MEASUREMENT time; delivery may be delayed or jittered by the
# emulator, so consumers must time-align against stamp, not arrival time.
builtin_interfaces/Time stamp
float64 r_v         # range relay -> vehicle [m]
float64 beta_v      # bearing relay -> vehicle in B [rad]
float64 r_t         # range relay -> target [m]
float64 beta_t      # bearing relay -> target in B [rad]
float64 sigma_r     # native range noise std [m]
float64 sigma_beta  # native bearing noise std [rad]
