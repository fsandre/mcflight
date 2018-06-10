# McFlight
McFlight stands for Mechanics of Flight and so far is only a translation of the F-16 model found at [Steven and Lewis book](https://www.amazon.com/Aircraft-Control-Simulation-Brian-Stevens/dp/0471371459), where the code is in Fortran. I reproduced it in Scilab scripts. The folder tree structure is explained below:

* atmosphere: implementation of [International Standard Atmosphere](https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19770009539.pdf) (ISA) model;
* eqm: not only the equations of motion but also the engine and aerodynamic data for the F16;
* sim: scripts of simulation and linearization examples;
* tests: scripts for testing some scripts of the other folders. In case of the atmospheric model, for example, this is only a check of expected values from the literature; for the aerodynamic/engine models, the interpolation is checked as well;
* trim: scripts for finding trim conditions, i.e., steady flight conditions.

Notice that the scripts shall be executed from the root folder.

This is a very first version of the toolset for control design and there are many points of potential improvement. Although for a quick check to see if all things are running I'd recommend:

* Run the tests. Setting the current directory to root folder of mcflight, execute:
```
--> exec('tests\aerodata_interpolation_f16.tst');
--> exec('tests\atmosphere.tst');
--> exec('tests\engine_f16.tst');
--> exec('tests\eqm_f16.tst');
--> exec('tests\trim_f16.tst');
```
* Run a script example which plots a comparison between linear and non-linear simulations of F-16 model:
```
--> exec('sim/plots_lin_nlin_f16.sce');
```

## Known issues
* The F-16 model trimmed power curve obtained by McFlight so far is not identical to the curve in Figure 3.6-3 of Stevens And Lewis. The throttle setting values obtained here are not corresponding to the ones in the book. This must be checked.

## TODO
* Lateral aerodynamic model needs to be well tested;
* Trimming for lateral-directional conditions (coordinated turn, for example);
