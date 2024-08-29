#!/usr/bin/env python

import rclpy
from rclpy.node import Node


from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller

from std_srvs.srv import Trigger

from spot_msgs.srv import ArmJointMove


class ArmJointMove(EventState):
        '''
        This state can be used for moving the arm joints by specifying values for each joint.

        -- joint_targets        float32[]       array for the 6 joint values

        <= continue             indicates successful completion of the state   
        <= failed               indicates failure to complete the execution of the state

        '''

        def __init__(self, joint_targets):
                # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
                super().__init__(outcomes = ['continue', 'failed'])

                ProxyServiceCaller.initialize(ArmJointMove._node)

                self._service_topic = '/spot1/arm_joint_move' # check the slash
                self._service = ProxyServiceCaller({self._service_topic: ArmJointMove})
                joint_targets_arr = [float(x) for x in joint_targets.split(',')]
                self._joint_targets = joint_targets_arr


        def execute(self, userdata):
                # This method is called periodically while the state is active.
                # Main purpose is to check state conditions and trigger a corresponding outcome.
                # If no outcome is returned, the state will stay active.

                request = ArmJointMove.Request()
                request.joint_targets = self._joint_targets

                try:
                  service_response = self._service.call(self._service_topic, request)
                  return 'continue'
                except:
                  return 'failed'
                pass

        def on_enter(self, userdata):
                # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
                # It is primarily used to start actions which are associated with this state.

                pass

        def on_exit(self, userdata):
                # This method is called when an outcome is returned and another state gets active.
                # It can be used to stop possibly running processes started by on_enter.

                pass # Nothing to do in this state.

        def on_start(self):
                # This method is called when the behavior is started.
                # If possible, it is generally better to initialize used resources in the constructor
                # because if anything failed, the behavior would not even be started.

                # rospy.wait_for_service(self._service_topic)
                pass

        def on_stop(self):
                # This method is called whenever the behavior stops execution, also if it is cancelled.
                # Use this event to clean up things like claimed resources.

                pass # Nothing to do in this state.