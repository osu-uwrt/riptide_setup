#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from riptide_flexbe_states.wait_killSwitch_state import WaitForKillSwitch
from riptide_flexbe_states.initial_depth_state import InitialDepthState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jun 25 2019
@author: Parth Parekh
'''
class startup_behaviorSM(Behavior):
	'''
	waits for kill switch then starts the rest of the statemachine
	'''


	def __init__(self):
		super(startup_behaviorSM, self).__init__()
		self.name = 'startup_behavior'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:95 y:101
			OperatableStateMachine.add('Wait on the kill Switch',
										WaitForKillSwitch(),
										transitions={'continue': 'Go To Depth', 'failed': 'Wait on the kill Switch'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:152 y:218
			OperatableStateMachine.add('Go To Depth',
										InitialDepthState(depth=5),
										transitions={'failed': 'Go To Depth', 'completed': 'finished'},
										autonomy={'failed': Autonomy.Off, 'completed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
