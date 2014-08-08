string topic
string type

# number of delivered messages during window
int32 delivered_msgs
# number of dropped messages during window
int32 dropped_msgs

# traffic during window, measured in bytes
int32 traffic

# mean/stddev/max period between two messages
duration period_mean
duration period_std
duration period_max

# mean/stddev/max age of message based on timestamps in the message header.
# In case the message does not have a header, it will be 0.
duration stamp_age_mean
duration stamp_age_std
duration stamp_age_max
