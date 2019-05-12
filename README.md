# Source code for master thesis


`autonomous_agent` - contains the economical agent for Robonomics network. Includes two nodes `trader_node` and `worker_node`

### trader_node

It's responsible for making offers for a corresponding demand

Options in the launch file:
* `lighthouse` - the lighthouse address in the Robonomics network
* `model` - unique model. DO NOT CHANGE 
* `token` - token to work with
* `order_lifetime` - how many blocks before deadline

### worker_node

It represents a simple proxy between the network and CPS. When a liability is created, the node tells the CPS to do all the work and then the worker_node sends a result to the network

### drone_control_system manager node

Implements a path planning system and watches after the drones during the fly
