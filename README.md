# Object-surface-exploration
This module is a solution for Task 4-1 of the TACMAN project. The robot incrementally constructs the surface of an object using tactile sensors. This module implements two solutions: one based on Gaussian process regression and one based on Gaussian process classification. For more details please read the README file.

## Dependencies
[iCub](http://wiki.icub.org/wiki/ICub_Software_Installation)

[GURLS](https://github.com/LCSL/GURLS)

[froce-cop-estimator](https://github.com/tacman-fp7/force-cop-estimator)

## How to compile
Use cmake to compile for your OS distribution. Under Linux you can compile it as follows:

```
mkdir build
cd build
ccmake ..
make install
```

## How to run 

```
object-surface-exploration --from <config-file>
```

### Control port (YARP rpc port)

```
/object-surface-exploration/rpc:i
```

commands:

`Help` display available commands
`explore <exploration-type> <object-name>` explores the workspace defined in the config file. Valid exploration-types are: regression, classification and multi 
 




## Publications
Jamali, N; Ciliberto, C ; Rosasco, L; Natale,~L (2016). ``Active perception: Building Objects' Models using Tactile Exploration''. In the proceedings of the IEEE-RAS International Conference on Humanoids.
