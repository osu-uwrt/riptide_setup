SMACH
=====

Remapping
---------

See template C_In from B_Result

https://github.com/JustBenj/riptide/blob/master/riptide_autonomy/state_machine/Main.py
http://wiki.ros.org/smach/Tutorials/User%20Data

What does Python's with do?



Navigation server is a node that should always be running, it just won't act until a action message is received.

.. code-block:: none

   call once at the end:
   actionServer.setAborted can be
      replaced with
                preempted
                success etc,
