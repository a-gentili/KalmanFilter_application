# Kalman Filter

This project aims at simulating a Kalman Filter that estimates and, if requested, predicts the current and future states of a particular system. The authors model two different systems: a user-mouse interaction and a mechanical system composed by a mass, which a randomly created force acts on. The implementation of the program uses allegro4.4 and pthread libraries beside an original library created to perform operations between matrices.


## Install dependencies

This application requires Allegro4 to run.

`sudo apt-get install liballegro4.4 liballegro4-4dev`

## Compile

`make`

## Launch

`sudo ./bin/main`

*Note: superuser privileges are needed to create real-time threads.*


