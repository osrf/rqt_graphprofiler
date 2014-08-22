string hostname
string ipaddress

# data applies during this time window.
# This information can be regarded as stale if the time since window_stop is
# more than twice the duration between window_start and window_stop
time window_start
time window_stop

# Total CPU and Memory used on this machine
int16 cpus_available
float32 cpu_load_mean
float32 cpu_load_std
float32 cpu_load_max

# mem_avialable
float64 phymem_used_mean
float64 phymem_used_std
float64 phymem_used_max
float64 phymem_avail_mean
float64 phymem_avail_std
float64 phymem_avail_max

NodeProfile[] nodes
