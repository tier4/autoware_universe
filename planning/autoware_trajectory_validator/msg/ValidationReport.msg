# This message represents a validation report for a trajectory.

# Possible levels of validation report severity.
uint8 OK = 1
uint8 WARN = 2
uint8 ERROR = 3

builtin_interfaces/Time trajectory_stamp
unique_identifier_msgs/UUID generator_id
string generator_name
uint8 level
MetricReport[] metrics
