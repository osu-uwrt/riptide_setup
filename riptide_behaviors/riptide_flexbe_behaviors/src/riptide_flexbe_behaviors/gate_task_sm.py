#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from riptide_flexbe_states.move_to_gate_state import MoveToGateState
from riptide_flexbe_states.align_gate_state import AlignGateState
from riptide_flexbe_states.move_through_gate_state import MoveThroughGateState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jul 03 2019
@author: Parth Parekh
'''
class gate_taskSM(Behavior):
	'''
	the gate behavior
	'''


	def __init__(self):
		super(gate_taskSM, self).__init__()
		self.name = 'gate_task'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:24 y:379, x:462 y:385
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:218 y:63
			OperatableStateMachine.add('Move To Gate',
										MoveToGateState(),
										transitions={'success': 'Align To Gate', 'failed': 'Move To Gate', 'command_error': 'failed'},
										autonomy={'success': Autonomy.Full, 'failed': Autonomy.Full, 'command_error': Autonomy.Full})

			# x:196 y:195
			OperatableStateMachine.add('Align To Gate',
										AlignGateState(),
										transitions={'success': 'Move Through Gate', 'command_error': 'failed'},
										autonomy={'success': Autonomy.Full, 'command_error': Autonomy.Full})

			# x:227 y:310
			OperatableStateMachine.add('Move Through Gate',
										MoveThroughGateState(),
										transitions={'success': 'finished', 'failed': 'Move Through Gate', 'command_error': 'failed'},
										autonomy={'success': Autonomy.Full, 'failed': Autonomy.Full, 'command_error': Autonomy.Full})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
