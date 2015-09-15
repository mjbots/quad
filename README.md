mjmech
======

Source and design files for various walking robots, their controlling
interfaces, and utilities for developing and operating them.

* GitHub https://github.com/mjbots/mjmech
* Most files are free hardware and software: Apache 2.0 License
* SemaphoreCI [![Build Status](https://semaphoreci.com/api/v1/projects/0a934ef2-b8f1-47d6-b1f2-1a68a4cf3982/540242/badge.svg)](https://semaphoreci.com/jpieper/mjmech)

Directory structure
-------------------

* **base/** - C++ source files common to many applications.
* **mech/** - C++ source files specific to walking robots.
* **python/** - Python bindings for various C++ classes.
* **legtool/** - A python application for experimenting with static
  walking gaits.
* **tools/** - Utilities for development and data analysis.
* **video-ui/** - Client application for FPV and real-time control.
* **configs/** - Configuration files for different robots and applications.
* **hw/** - Hardware design files along with firmware.
