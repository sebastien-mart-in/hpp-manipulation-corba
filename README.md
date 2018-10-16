# hpp-manipulation-corba

[![Building Status](https://travis-ci.org/humanoid-path-planner/hpp-manipulation-corba.svg?branch=master)](https://travis-ci.org/humanoid-path-planner/hpp-manipulation-corba)
[![Pipeline status](https://gepgitlab.laas.fr/humanoid-path-planner/hpp-manipulation-corba/badges/master/pipeline.svg)](https://gepgitlab.laas.fr/humanoid-path-planner/hpp-manipulation-corba/commits/master)
[![Coverage report](https://gepgitlab.laas.fr/humanoid-path-planner/hpp-manipulation-corba/badges/master/coverage.svg?job=doc-coverage)](http://projects.laas.fr/gepetto/doc/humanoid-path-planner/hpp-manipulation-corba/master/coverage/)

This package is part of the [HPP] software and extends the functionalities of [hpp-corbaserver].
It implements a CORBA server in order to use the functionalities of package [hpp-manipulation] in Python.

### Version
1.0

### Dependencies

[hpp-manipulation-corba] needs the following package to be installed:

* [hpp-core]
* [hpp-corbaserver]
* [hpp-wholebody-step-corba]
* [hpp-manipulation]
* [hpp-manipulation-urdf]

### Installation

Make sure you have installed all the dependency.

```sh
$ git clone https://github.com/humanoid-path-planner/hpp-manipulation-corba
$ cd hpp-manipulation-corba
$ mkdir build && cd build
$ cmake ..
$ make install
```

### Todo's

* Enhance Python class ConstraintGraph.

[hpp-core]:https://github.com/humanoid-path-planner/hpp-core
[HPP]:https://github.com/humanoid-path-planner/hpp-doc
[hpp-constraints]:https://github.com/humanoid-path-planner/hpp-constraints
[hpp-corbaserver]:https://github.com/humanoid-path-planner/hpp-corbaserver
[hpp-wholebody-step-corba]:https://github.com/humanoid-path-planner/hpp-wholebody-step-corba
[hpp-manipulation]:https://github.com/billx09/hpp-manipulation
[hpp-manipulation-corba]:https://github.com/billx09/hpp-manipulation
[hpp-manipulation-urdf]:https://github.com/billx09/hpp-manipulation-urdf
[hpp-model]:https://github.com/humanoid-path-planner/hpp-model
[hpp-util]:https://github.com/humanoid-path-planner/hpp-util
