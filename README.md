# Hexapod
My Hexapod source code repository.

The robot was design in order to participate to the french robotic cup. But the first step before a full autonomus robot is to design one remootely controlled.

## Mecanic
The mecanic of the robot can be found on Onshape, under the project name : "Hexapod Ironcrabs"
Or if your lazy and don't want to search, here is the link : [Hexapod Ironcrabs on Onshape](https://cad.onshape.com/documents/e26a3b7c69fc81b4256c14c0/w/d2bfec1e1ae743a3e4b564fc/e/f59567beb0b7a79da3099916)

For now the mecanic is really simple : six identical legs attach to the body, each of them composed to three servomotors (SCS15) and one sensor board (LegBoard) at the extremity. A shield is attached to this board to detect shocks with the environement.

![Awesome hexapod robot](https://raw.githubusercontent.com/Titwin/Hexapod/master/RobotImage.png)

PS : I'm actually working on a new version of the mecanic so The software is not yet related to the CAO version.

## Electronic
All the electronic board (exept sensor board or Raspberry Pi), was developped on [CircuitMaker](https://workspace.circuitmaker.com/)
There is several boards on this project, so several CircuitMaker project was created (one for each).

The architecture is quite simple : the power distribution (powering master, hub slave and all legs daisy chains), is done by the Hub slave board. All slaves are connected on a unique TTL bus (V+, GND, and Data).

TODO : incrust diagram image.

## Software
Since we have several electronic board with a microcontroller on each, we have to separate and explain separately the software

### Master
The Master, which here is embeded in a Raspberry Pi 3, is responsible to make everythings working.
His jobs are :
* Read state of all slaves (motors position, torque, legs forces, ...),
* Write on slaves registers to change their tasks or workflow (motors goal position),
* Read inputs from user (via the gamepad), and update the internal goal state of the robot (the walking state machine),
* Provide a wifi hotspot and SSH server (for debug),
* Run a webserver to provide an awsome debug tool (the casual god-mode),
* ...

### Slaves
As explained previously, a slave is an autonomous electronic board, which is responsible to a time critical task.
Indeed the master is depending on an OS (here Linux), so tasks like motor control or sensors integration are not acurate on these architecture.
My solution was to create multiple board with a microcontroller dedicated to these critical tasks. But for the most important task, I'v found a very good servomotor reference SCS15 by Feetech.
The protocole was herited from the Servomotor protocol.

#### SCS15 servomotors
Its just a servomotor.

#### LegBoard slaves
This board is responsible to / capable of :
* changing an RGB LED color, blink mode,
* reading and calibrate a force sensor at the leg extremity,
* reading and decoding shoks direction on a 3D printed shield,
* reading distance in front of the leg using a ToF sensor (the cool pololu one),
* some electronic component for debuging and power decoupling TTL bus.

#### Hub slave
This board is responsible to / capable of :
* display some debug information using a little Oled screen,
* enable/disable power distribution on a leg,
* reading the power consuption on each legs,
* reading the battery voltage for autonomy extrapolation,
* some electronic component for debuging and power decoupling TTL bus,
* powering on/off a fan to cool down all the electronic in the body.




