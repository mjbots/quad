# Getting started with the quad A1 #

## Safety ##

The mjbots quad A1 is a high performance robot capable of very fast
and powerful motions.  It is not recommended for use in the vicinity
of untrained personnel and especially not children or others who do
not appreciate what it is capable of.

Minimize physical handling of the robot while the power switch is on,
as malfunctions may cause legs to to move rapidly, creating pinch
hazards.

## Turn on ##

1. Ensure the switch is in the "off" position.  When off, the LED side of the switch is *not* depressed.

2. Insert a 4Ah ONE+ battery.  It may be necessary to apply a moderate tap with your fist in order to get both retaining clips to latch.

3. Flip the quad A1 over to be right side up and extend legs to be maximally stretched out, flat against the ground.  This is the "turn-on" position.
  * NOTE: The software is able to detect *some but not all* invalid
    turn-on positions.

4. Activate the power switch.  This means pressing the side with the LED down.

5. After ~30s, the robot's wifi access point will become available.  Connect to it.

6. Point your web browser to http://192.168.16.47:4778

## Joystick ##

The web control application supports a connected gamepad, which is the
simplest means of controlling the robot.  The "Y/4" button on the
gamepad enables mode select, while the d-pad up and down allow you to
cycle between modes.  Once you pick a mode that is not "Stop", the
quad A1 will stand up.

While moving, the left analog stick controls translation, while the
left and right motion of the right analog stick controls yaw.

When stopped, the left trigger can be pulled to enable "emote mode".
In that mode, the analog sticks control the attitude of the body.

## Keyboard ##

It is also possible to operate the robot with a mouse and keyboard.
The mouse can be used to click and select a mode.  WASDQE keys can be
used to pilot the quad A1 as per the on-screen display.

# Maintenance #

## Belt tension ##

The leg belts need to be kept appropriately tensioned to minimize the
possibility of belt skipping, while not overly stressing the 3d
printed upper leg structure.  The M3 adjustment bolt in each leg can
be used to adjust that tension.  As the belts stretch with age, it
will likely be necessary to use a longer bolt than the M3x12 initially
installed.  M3x14 is the longest suitable bolt.

When tensioned properly,

## Rezeroing leg ##

If the leg is disassembled or re-assembled, or the belt skips, then that leg will need to be re-zeroed.  This can be accomplished as follows:

1. Lay the quad A1 upside down on a flat surface.
2. Install the leg zeroing fixtures on the leg in question.  The lower
   leg qdd100 rests on the curved part of the large fixture, and the
   upper part restrains the upper leg.
3. The smaller fixture registers the upper and lower leg together.
   Slide it down as far as possible, then attempt to get the lower leg
   as centered as possible in any remaining play.
4. Connect to the robot and stop the existing software:

```
ssh -i mjmech/utils/ssh/mjbots.ssh pi@192.168.16.47
sudo screen -r
<CTRL-c>
```

5. Determine the leg number of the leg to be rezeroed.  When looking
   from the top, the legs are numbered as follows (note the robot is
   upside down now, so this will be mirrored):

```
   1      2

     FRONT

     BACK

   3      4
```

6. Determine which servos are associated with that leg:

```
Leg 1: 1-3
Leg 2: 4-6
Leg 3: 7-9
Leg 4: 10-12
```

7. Rezero the leg, using the servo numbers from above.

```
./moteus_tool -t1-3 --zero-offset
```

8. If more legs need to be rezeroed, the fixture can be moved between
   legs and the step 7 command run more as necessary.

9. Shutdown the computer, wait 15s, then turn off the power switch.

```
shutdown -h now
```

10. At this point the quad A1 can be set in the "turn-on" position and
    started up as normal.

## Upgrading software ##

1. Compile:

```
tools/bazel build --config pi -c opt //mech:quad_deploy.tar
```

2. Copy it to the robot:

```
rsync -Pv -e "ssh -i utils/ssh/mjbots.ssh"  \
  bazel-out/armeabihf-opt/bin/mech/quad_deploy.tar pi@192.168.16.47:
```

3. Perform the upgrade and restart the software.

NOTE: If any configurations have been modified, you will want to
manually capture those now, as this process will reset all
configuration to factory default.

```
ssh -i utils/ssh/mjbots.ssh pi@192.168.16.47
sudo screen -r
<CTRL-c>
cd /home/pi
sudo -u pi tar xvf quad_deploy.tar
sync
cd mech
./start-robot.sh
<CTRL-a d>
```
