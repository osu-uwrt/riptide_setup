#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from riptide_flexbe_states.move_action_state import MoveActionState
from riptide_flexbe_states.align_action_state import AlignActionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jun 26 2019
@author: Parth Parekh
'''
class GateTaskSM(Behavior):
	'''
	Does Gate Task
	'''


	def __init__(self):
		super(GateTaskSM, self).__init__()
		self.name = 'Gate Task'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:690 y:663, x:719 y:166
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:140 y:61
			OperatableStateMachine.add('Turn Towards Gate',
										MoveActionState(dishes_to_do=oof),
										transitions={'success': 'Move Towards Gate', 'failed': 'Turn Towards Gate', 'command_error': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off, 'command_error': Autonomy.Off})

			# x:184 y:163
			OperatableStateMachine.add('Move Towards Gate',
										MoveActionState(dishes_to_do=d),
										transitions={'success': 'Align With Gate', 'failed': 'Move Towards Gate', 'command_error': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off, 'command_error': Autonomy.Off})

			# x:223 y:279
			OperatableStateMachine.add('Align With Gate',
										AlignActionState(),
										transitions={'success': 'Move forwards', 'command_error': 'Align With Gate'},
										autonomy={'success': Autonomy.Off, 'command_error': Autonomy.Off})

			# x:79 y:381
			OperatableStateMachine.add('Move forwards',
										MoveActionState(dishes_to_do=3),
										transitions={'success': 'rotate 90 degrees', 'failed': 'Move forwards', 'command_error': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off, 'command_error': Autonomy.Off})

			# x:119 y:472
			OperatableStateMachine.add('rotate 90 degrees',
										MoveActionState(dishes_to_do=3),
										transitions={'success': 'pitch 90 degrees', 'failed': 'rotate 90 degrees', 'command_error': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off, 'command_error': Autonomy.Off})

			# x:306 y:495
			OperatableStateMachine.add('pitch 90 degrees',
										MoveActionState(dishes_to_do=3),
										transitions={'success': '90 deg rot', 'failed': 'pitch 90 degrees', 'command_error': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off, 'command_error': Autonomy.Off})

			# x:455 y:549
			OperatableStateMachine.add('90 deg rot',
										MoveActionState(dishes_to_do=3),
										transitions={'success': 'go through gate', 'failed': '90 deg rot', 'command_error': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off, 'command_error': Autonomy.Off})

			# x:627 y:519
			OperatableStateMachine.add('go through gate',
										MoveActionState(dishes_to_do=4),
										transitions={'success': 'finished', 'failed': 'go through gate', 'command_error': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off, 'command_error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
