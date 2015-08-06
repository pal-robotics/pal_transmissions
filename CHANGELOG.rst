^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pal_transmissions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2015-08-06)
------------------
* Add custom REEM/REEM-C head transmission
  Transmission involving both head joints. Works like two independent reducers,
  except that the tilt joint's position limits depend on the pan joint's actual
  position.
  Implementation includes new transmission class, URDF loader plugin and test
  suite.
* Contributors: Adolfo Rodriguez Tsouroukdissian
