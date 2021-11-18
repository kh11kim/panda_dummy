# panda_dummy

1. Catkin build, Source devel/setup.bash
2. Run dummy_node.py 
* rosrun panda_dummy dummy_node.py
3. If you do "rostopic list", you can see two topics
* /peginhole_request: This is request topic to panda. 
* /peginhole_result: This is result topic for peg-in-hole task.
4. Check /peginhole_result topic (in a different shell)
* rostopic echo /peginhole_result
5. Publish a sample topic of peginhole request, and see how the node and result topic works
* rostopic pub /peginhole_request <tab> <tab> <enter>
6. /peginhole_result gives a three values
* int state (0: Idle, 1: Running, 2: Done)
* bool is_done (bool)
* int errorcode (TBD)
