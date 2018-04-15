# McFlight
McFlight stands for Mechanics of Flight and so far is only a translation of the F-16 model found at [Steven and Lewis book](https://www.amazon.com/Aircraft-Control-Simulation-Brian-Stevens/dp/0471371459), where the code is in Fortran. I reproduced it in Scilab scripts. The folder tree structure is explained below:

* atmosphere: implementation of [International Standard Atmosphere](https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19770009539.pdf) (ISA) model;
* eqm: not only the equations of motion but the engine and aerodynamic data for the F16;
* sim: scripts of simulation and linearization examples;
* tests: scripts for testing some scripts of the other folders. In case of the atmospheric model, for example, this is only a check of expected values from the literature; for the aerodynamic/engine models, the interpolation is checked as well;
* trim: scripts for finding trim conditions, i.e., steady flight conditions.

Notice that the scripts shall be executed from the root folder.
