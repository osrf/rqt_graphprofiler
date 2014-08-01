string hostname
string ipaddress

# data applies during this time window
time window_start
time window_stop

# Total CPU and Memory used on this machine
int16 cpus_available
float32 cpu_total_load_mean
float32 cpu_total_load_std
float32 cpu_total_load_max

# mem_avialable
int32 mem_total
float32 mem_percent_used

NodeProfile[] nodes
