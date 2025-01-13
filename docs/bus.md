# Bus Information

The bus is an Open-Drain bus. The bus is pulled high by an external pull up resistor then pulled low by the device when it wants to communicate. If the bus is low then it is busy, so a simple collision detection is to make sure the bus is high before writing to it.

## Hardware

If the MCU doesn't have native support for Open-Drain busses, then two n-channel mosfets can be used. One where the MCU controlls the gate and the bus is on drain for TX from the MCU, and the other where the MCU pin is on drain and the bus is on gate for the RX to the MCU.
