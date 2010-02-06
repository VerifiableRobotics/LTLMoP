Configuration and Data Files
============================

Definitions
-----------

* A :

* A :doc:`robot_file` describes the available functionalities of a given class of robot (i.e., sensor and actuator propositions), as well as the motion control strategy that should be used for driving the robot from region to region (e.g., potential field + differential-drive feedback linearization).

* A :doc:`spec_file` contains a specification, written in Structured English, which describes how the robot should behave.

* An :doc:`aut_file` is generated automatically from a :doc:`spec_file` and contains an automaton whose execution will cause the robot to satisfy the original specification (under environmental assumptions).

* An :doc:`exp_config` 


Example
-------

To perform an experiment, the following steps should serve as a reasonable guideline:

#. Select a robot and import it into :doc:`specEditor`


Contents
--------

.. toctree::
   :maxdepth: 2

   exp_config
