# This message is mainly used for convenience purposes when interacting with the graph slam from outside the graph slam node, i.e. playing back data from python scripts etc.

string robot_name
bool initialized
bool in_optimization
bool in_loop_closure
bool in_graph_exchange

# Graph exchange statistics (populated when in_graph_exchange transitions to false)
int32 last_exchange_keyframes
int32 last_exchange_edges
int32 last_exchange_bytes
int32 total_received_bytes