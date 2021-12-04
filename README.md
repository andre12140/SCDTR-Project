# SCDTR-Project

Real-Time Cooperative Decentralized Control of Illumination Systems

The objective of this project is to design a real-time control system for a distributed illumination system in a small-scale model of an office space. Conceptually, each desk has a smart luminaire comprising a light emitting device (LED), a luminance sensor, a presence sensor, as well as computational and communication elements.

In the actual project we will simulate the office with a small opaque cardboard box and each luminaire consists of a breadboard with the Arduino, LED and LDR circuits.

The objective is to minimize the energy consumption and maximize user comfort. Energy minimization will be achieved by controlling the dimming level of each LED such that occupied desks have luminance levels above a certain value (HIGH) and unoccupied ones have a luminance level above a lower value (LOW).

User comfort should be maximized by keeping the illumination always above or equal to the minimum levels (visibility), while minimizing the up-and-down variations of the illuminance (flicker) during desk occupation. These variations may be due to noise, external disturbances, or interference caused by the other luminaires in the shared space. Noise and external disturbances can be compensated by a local feedback control loop at each luminaire, but internal disturbances due to interference from other desks can be predicted and compensated through proper communication and synchronization between luminaires (global control).
