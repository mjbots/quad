# mjmech geometry #

## Unit naming conventions ##

Quantities expressed in a certain unit should have that unit included
as a suffix that variable name.

## Frame naming conventions ##

Quantities which are expressed relative to some frame or frames have those frames suffixed on the variable name using the simbody conventions :https://simtk.org/docman/view.php/47/231/SimbodyTheoryManual.pdf

As there, common frames may be abbreviated with a single uppercase
letter.

## Frame conventions ##

Coordinate systems are right handed, if there is a human associated
direction, they should be interpreted as:

* x is positive forward
* y is positive right
* z is positive down

## Common frames ##

Joint Frame (J): Each joint has a frame located at the center of
rotation of the shoulder, with +x facing outward, +y facing right, +z
facing down.

Leg Frame (G): The leg frame is located at the center of the shoulder
joint, with +x facing forward, +y facing right, and +z facing down.

Body Frame (B): This frame is oriented at the center of the quadruped
body and fixed to the chassis.

Robot Frame (R): This frame is nominally at the center of the
quadruped body and oriented identically, but may be offset or rotated
to account for the robot body shifting relative to its feet.

Local Frame (L): This frame is oriented such that down is parallel to
gravity.  Other degrees of freedom are initialized with the quadrupeds
body center at turn on, and have no absolute correlation with the
outside world.
