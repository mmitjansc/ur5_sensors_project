# portaudio_ros

```portaudio_ros.cpp``` reads data from the */pressure* topic and outputs the impulse response of an underdamped 2nd order 
system every time it is in the state *grasping*. It starts in the *resting* state.

It has a built-in state machine, that works as follows:

    1. S0: *Resting* state, gripper open.
    2. S1: *Grasping* state. Reached when coming from S0 and a sensor detects a significant pressure. This state outputs
       the impulse response through the jack port (to drive the voice coil), and jumps to S1.
    3. S2: *Holding* state. Reached automatically from S1, and remains in this state until the gripper opens (i.e. no sensor
       is detecting any pressure), in which case goes back to S0.

