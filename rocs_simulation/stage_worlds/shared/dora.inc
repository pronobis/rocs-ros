# Pioneer robots
include "shared/pioneer.inc"

# Sick laser
include "shared/sick.inc"


define dora pioneer3dx
(
  sicklaser( samples 361 laser_sample_skip 4 pose [0.1 0 0 0] )
)
