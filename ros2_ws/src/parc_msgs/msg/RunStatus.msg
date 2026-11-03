builtin_interfaces/Time stamp
uint8 IDLE       = 0      # no run in progress
uint8 ARMED      = 1      # order sent, waiting for start
uint8 RUNNING    = 2      # clock running
uint8 FINISHED   = 3      # last tag, finish msg or judge
uint8 TIMED_OUT  = 4      # 10 minutes reached
uint8 TERMINATED = 5      # stopped by a judge for safety
uint8 state
float32 elapsed_s
float32 time_limit_s      # 600
uint8[] tag_order
uint8[] tags_reported     # received, not judged
uint8 start_method        # StartSignal.METHOD_*
bool false_start          # moved before the start signal
