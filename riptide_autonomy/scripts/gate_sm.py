#!/usr/bin/env python

import rospy
import smach
import smach_ros
import qualify_sm

class SetFinalGateOutcome(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['entered_qualify_gate',
                                    'entered_casino_gate',
                                    'exited_qualify_gate'],
                            input_keys=['entered_qualify_gate_in',
                                        'entered_casino_gate_in',
                                        'exited_casino_gate_in',
                                        'gate_type_in'],
                            output_keys=['entered_qualify_gate_out',
                                        'entered_casino_gate_out',
                                        'exited_casino_gate_out'])

    #Adjust userdata fields and set final outcome accordingly
    def execute(self, userdata):
        if self.userdata.gate_type_in = "qualify"
            if self.userdata.entered_qualify_gate_in == 0
                self.userdata.entered_qualify_gate_out = 1
                return 'entered_qualify_gate'

            elif self.userdata.exited_qualify_gate_in == 0
                self.userdata.exited_qualify_gate_out = 1
                return 'exited_qualify_gate'

        elif self.userdata.gate_type_in = "casino"
            if self.userdata.entered_casino_gate_in == 0
                self.userdata.entered_casino_gate_out = 1
                return 'entered_casino_gate'

def main():
    gate_sm = smach.StateMachine(outcomes=['entered_qualify_gate','entered_casino_gate','exited_qualify_gate'],
                                input_keys=['prev_completion_in',
                                            'gate_type_in',
                                            'casino_color_in'],
                                output_keys=['prior_completion_out'])
    gate_sm.userdata.entered_qualify_gate = 0
    gate_sm.userdata.exited_qualify_gate = 0
    gate_sm.userdata.entered_casino_gate = 0
    gate_sm.userdata.depth = 0
    gate_sm.userdata.euler_rpy = [0,0,0]

    #Remap userdata to each action state
    with gate_sm:
        smach.StateMachine.add('FIND_GATE',
                                SimpleActionState('FindServer',
                                                    FindGateAction,
                                                    goal_slots=['gate_type',
                                                                'casino_color']),
                                transitions={'succeeded':'GATE_ALIGNMENT'},
                                remapping={'gate_type':'gate_type_in',
                                            'casino_color':'casino_color_in'})
        smach.StateMachine.add('GATE_ALIGNMENT',
                                SimpleActionState('AlignmentServer',
                                                    GateAlignmentAction,
                                                    goal_slots=['gate_type',
                                                                'casino_color'],
                                                    result_slots=['depth', 'euler_rpy']),
                                transitions={'succeeded':'PASS_THRU_GATE'},
                                remapping={'gate_type':'gate_type_in',
                                            'casino_color':'casino_color_in',
                                            'depth':'depth', 'euler_rpy':'euler_rpy'})
        smach.StateMachine.add('PASS_THRU_GATE',
                                SimpleActionState('PassThruGateServer',
                                                    PassThruGateAction,
                                                    goal_slots=['gate_type', 'casino_color',
                                                                'depth','euler_rpy']),
                                transitions={'succeeded':'SET_FINAL_GATE_OUTCOME'},
                                remapping={'gate_type':'gate_type_in',
                                            'casino_color':'casino_color_in',
                                            'depth':'depth', 'euler_rpy':'euler_rpy'})
        smach.StateMachine.add('SET_FINAL_GATE_OUTCOME', SetFinalGateOutcome(),
                                transitions={'entered_qualify_gate':'entered_qualify_gate',
                                            'entered_casino_gate':'entered_casino_gate',
                                            'exited_qualify_gate':'exited_qualify_gate'}
                                remapping={'entered_qualify_gate_in':'entered_qualify_gate',
                                            'entered_casino_gate_in':'entered_casino_gate',
                                            'exited_qualify_gate_in':'exited_qualify_gate',
                                            'gate_type_in':'gate_type',
                                            'entered_qualify_gate_out':'entered_qualify_gate',
                                            'entered_casino_gate_out':'entered_casino_gate',
                                            'exited_casino_gate_out':'exited_qualify_gate'})

    outcome = gate_sm.execute()

if __name__ == "__main__"
    main()
