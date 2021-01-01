mjbots quad
===========

Source and design files for the mjbots quad series of robots, their
controlling interfaces, and utilities for developing and operating
them.

* GitHub https://github.com/mjbots/mjmech
* Most files are free hardware and software: Apache 2.0 License
* Overall F360 assembly: https://a360.co/2Xot0VJ

Directory structure
-------------------

* **base/** - C++ source files common to many applications.
* **ffmpeg/** - C++ ffmpeg wrappers.
* **gl/** - C++ GL wrappers.
* **mech/** - C++ source files specific to walking robots.
* **simulator/** - C++ source files for a quadruped simulator.
* **utils/** - Utilities for development and data analysis.
* **configs/** - Configuration files for different robots and applications.
* **hw/** - Hardware design files along with firmware.
* **docs/** - Documentation.


First Time Setup
----------------

The following should work on Ubuntu 20.04

```
./install-packages
```

Building for host
-----------------

```
tools/bazel test //...
```

Running simulation
------------------

```
./bazel-bin/simulator/simulator -c configs/quadruped.ini
```

Then point your web browser to `http://localhost:4778`
