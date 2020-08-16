mjmech
======

Source and design files for various walking robots, their controlling
interfaces, and utilities for developing and operating them.

* GitHub https://github.com/mjbots/mjmech
* Most files are free hardware and software: Apache 2.0 License
* buildkite [![Build status](https://badge.buildkite.com/0689c3a0d219ec219be55d6a233ecba2a01c8bb165c1a49a68.svg?branch=master)](https://buildkite.com/mjbots/mjmech)

Directory structure
-------------------

* **base/** - C++ source files common to many applications.
* **ffmpeg/** - C++ ffmpeg wrappers.
* **gl/** - C++ GL wrappers.
* **mech/** - C++ source files specific to walking robots.
* **simulator/** - C++ source files for a quadruped simulator.
* **utils/** - Utilities for development and data analysis.
* **video-ui/** - Client application for FPV and real-time control.
* **configs/** - Configuration files for different robots and applications.
* **hw/** - Hardware design files along with firmware.
* **docs/** - Documentation.


First Time Setup
----------------

The following should work on Ubuntu 18.04

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
