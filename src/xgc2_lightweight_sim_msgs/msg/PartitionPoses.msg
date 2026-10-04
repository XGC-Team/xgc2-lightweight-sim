# One latest complete frozen partition; each record owns its original source stamp.
# No batch stamp can refresh a sibling record. Not the scientific state FIFO.
string session_id
string partition_id
NamedPose[] poses
