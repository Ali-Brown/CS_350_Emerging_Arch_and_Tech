# Summarize the project and what problem it was solving.
The thermostat prototype according to the project specification requires a microcontroller with support for GPIO, I2C, and UART peripherals. GPIO is required to indicate temperature state (heat on or off) of the thermometer through LED and to adjust (increase or decrease) temperature set-point for the thermometer by GPIO interrupts on two buttons. I2C is required to read room temperature from a temperature sensor. UART is required to display or output the thermostat status (room-temperature, set-point, heating-state, and seconds) to simulate data to be sent to the server. 

# What did you do particularly well?
Generating the task scheduler include three state machines (processing temperature, adjust room set state, and display status) all synched to carry out project specification. The scheduler was particularly helpful in writing the codes. 

# Where could you improve?
Although the thermostat prototype worked as expected, I could have improved in my implementation of the GPIO interrupts on the two buttons and perhaps state transitions in the state machines.

# What tools and/or resources are you adding to your support network?
As someone new to embedded systems programming and microcontrollers, the TI documentations on the CC3220 SimpleLinkTM Wi-Fi® LaunchPadTM, GPIO, I2C, and UART helped a lot.  

# What skills from this project will be particularly transferable to other projects and/or course work?
The use of task scheduler and state machine will be very useful in planning any complex project.

# How did you make this project maintainable, readable, and adaptable?
Through the use of the task scheduler, use of descriptive variable and function names, use of state transitions and actions, and comments. 
