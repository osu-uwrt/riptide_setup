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
                            input_keys=['gate_type_in',
                                        'prev_completion_in'],
                            output_keys=['prev_completion_out'])

    #Set final outcome accordingly and return corresponding outcome
    def execute(self, userdata):
        if self.userdata.gate_type_in = "qualify"
            if self.userdata.prev_completion_in == "nothing"
                self.userdata_prev_completion_out = "entered_qualify_gate"
                return 'entered_qualify_gate'

            elif self.userdata.prev_completion_in == "circled_the_marker"
                self.userdata.prev_completion_out = "exited_qualify_gate"
                return 'exited_qualify_gate'

        elif self.userdata.gate_type_in = "casino"
            if self.userdata.prev_completion_in == "nothing"
                self.userdata.prev_completion_out = "entered_casino_gate"
                return 'entered_casino_gate'

def main():
    gate_sm = smach.StateMachine(outcomes=['entered_qualify_gate','entered_casino_gate','exited_qualify_gate'],
                                input_keys=['prev_completion_in',
                                            'gate_type_in',
                                            'casino_color_in'],
                                output_keys=['prev_completion_out'])
    gate_sm.userdata.depth = 0
    gate_sm.userdata.euler_rpy = [0,0,0]

    #Remap userdata to each action state via goal_slots
    #Remap userdata from an action state to the state machine's userdata via result_slots
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
                                remapping={'gate_type_in':'gate_type_in',
                                            'prev_completion_in':'prev_completion_in',
                                            'prev_completion_out':'prev_completion_out'})

    outcome = gate_sm.execute()

if __name__ == "__main__"
    main()
