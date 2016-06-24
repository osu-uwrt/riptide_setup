Actuators
=========

Riptide is able to move and complete the various tasks of the RoboSub competition through the use of several actuators mounted to the chassis of the vehicle.


Thrusters
---------

Ten `T200`_ thrusters from `Blue Robotics`_ are utilized on Riptide and give it the ability to move with six degrees of freedom. The thrusters are fixed to the water-jet chassis, four horizontal, four vertical, and two sideways. The thruster wires were cut to length, soldered, and resin was injected in molds to seal the electrical connections.


Pneumatics
----------

The pneumatics system is controlled by eight 3-way electronic solenoid valves. The valves are part of a manifold to reduce the number of connections as well as make the system more space efficient. The pneumatics system is kept isolated in the pneumatics auxiliary housing located near the bottom of the vehicle.

Air is supplied by an onboard, externally mounted, air cylinder. The pressure is regulated down to the working pressure of 38 psi to allow a constant working pressure until the tank is nearly depleted. Air lines pass through a custom reinforced bulkhead that supports the use of off the shelf fittings.


Grippers
~~~~~~~~

There are two manipulators on Riptide, located on either side of the vehicle along the forward housing compartment. The claws were designed to reach as far as possible while still maintaining a low profile when not in use. To do this, a two way linear actuator was used. The claw arms were waterjet and Delrin inserts were used around the screws to reduce friction during actuation. Additionally, stainless steel rods with a slider were used to reduce any side loading. A CAD rendering of one of the claws can be seen in Fig 10.


Marker Droppers
~~~~~~~~~~~~~~~

There are two marker droppers aboard Riptide, located on either side underneath the central cylinder of the housing. Each marker dropper is operated by a permanent magnet, a steel 1 in diameter ball, and compressed air. The steel marker ball rests in a cylindrical chamber and is separated from the magnet by a 1/8” Delrin plate. The plate is thin enough that the magnet can keep the ball still, yet it is thick enough that firing compressed air through a check-valve at the top of the chamber can overcome the magnetic force and release the ball. Gaskets are used to water-tight seal the magnet. A gasket is placed between the magnet and the top plate and between the magnet and the magnet face plate.

Other designs were considered, but this design was ultimately chosen due to simplicity. Another design required pneumatic controls which would have worked, yet required  additional hardware for the AUV. The final physical design was chosen based on machinability. Other previous designs were found to be too difficult to machine or too difficult to replicate more than once if something went wrong with the first part. The use of Delrin as the main component material kept the part lightweight, easy to machine, and inexpensive.


Torpedo Launchers
~~~~~~~~~~~~~~~~~

The torpedo launchers consist of two Delrin barrels held in place by two Delrin support structures, shown in Fig 11. The launchers are attached at the top of either side of Riptide’s forward housing compartment. Two torpedoes are launched from the barrels through the internal pressurization of the torpedoes themselves. At the pneumatic pressure’s peak, the torpedo slips passed the O-ring keeping it attached to the pneumatic valve and speeds forward through the water. The torpedoes consists of a 1.5-inch diameter head attached to a 0.75-inch diameter shaft that has 3 stability fins attached at its base. The 5-inch long torpedo fits snugly in the 5.5-inch long barrel to assure that all air pressure is devoted to pushing the torpedo out of the barrel and so the torpedo does not wobble prior to exit.

The initial stages of the torpedo launcher’s development began with extensive research into approaches of past successful teams. It was found that pneumatics seemed to be the most feasible option for success.

After finalizing the pursuance of a pneumatic launcher system, the next step was to determine how to pressurize the mechanism. Several methods were proposed including pressurizing the barrel external to the torpedo. Finally, the decision was made to pressurize the inside of the torpedo itself to eliminate the friction vs. effective sealing decision of the torpedo-barrel contact.

Most of the machining was done on a lathe including the barrels (turned and bored), the torpedo head (turned and bored), and the torpedo shaft (faced). The support structures for the barrels were water-jet by a third party.


.. _Blue Robotics: https://www.bluerobotics.com/
.. _T200: http://docs.bluerobotics.com/thrusters/t200/
