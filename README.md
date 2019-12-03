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
* **mech/** - C++ source files specific to walking robots.
* **python/** - Python bindings for various C++ classes.
* **legtool/** - A python application for experimenting with static
  walking gaits.
* **utils/** - Utilities for development and data analysis.
* **video-ui/** - Client application for FPV and real-time control.
* **configs/** - Configuration files for different robots and applications.
* **hw/** - Hardware design files along with firmware.


First Time Setup
----------------

The following should work on Ubuntu 18.04

```
./install-packages
```

Building
--------

```
tools/bazel test //...
```
