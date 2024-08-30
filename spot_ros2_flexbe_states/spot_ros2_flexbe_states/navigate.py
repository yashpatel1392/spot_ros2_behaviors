#!/usr/bin/env python

import rclpy
from rclpy.node import Node


from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from std_srvs.srv import Trigger

from spot_msgs.action import NavigateTo, NavigateToAction, NavigateToGoal
from actionlib_msgs.msg import GoalStatus


class Navigate(EventState):
        '''
        This state can be used for navigating the spot to a waypoint.

        -- goal_waypoint        string          waypoint id for the waypoint where the spot should navigate to  
        -- init_waypoint        string          waypoint id for the waypoint where the spot should localize
        -- upload_path          string          path to the directory where the graph is present

        #> None
        ># None

        <= continue             indicates successful completion of the state   
        <= failed               indicates failure to complete the execution of the state

        '''

        def __init__(self, spot_name, goal_waypoint, init_waypoint, upload_path):
            # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
            super().__init__(outcomes = ['continue', 'failed'])

            ProxyActionClient.initialize(Navigate._node)

            self._upload_path = upload_path
            self._navigate_to = goal_waypoint
            self._init_waypoint = init_waypoint

            self._action_topic = '/' + spot_name + '/navigate_to'
            self._client = ProxyActionClient({self._action_topic: NavigateToAction})


        def execute(self, userdata):
            # This method is called periodically while the state is active.
            # Main purpose is to check state conditions and trigger a corresponding outcome.
            # If no outcome is returned, the state will stay active.

            if self._client.has_result(self._action_topic):
                status = self._client.get_state(self._action_topic)
            if status == GoalStatus.SUCCEEDED:
                return 'continue'
            elif status in [GoalStatus.PREEMPTED, GoalStatus.REJECTED,
                            GoalStatus.RECALLED, GoalStatus.ABORTED]:
                return 'failed'


        def on_enter(self, userdata):
            # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
            # It is primarily used to start actions which are associated with this state.

            goal = NavigateToGoal()
            goal.upload_path = self._upload_path
            goal.navigate_to = self._navigate_to
            goal.initial_localization_fiducial = False
            goal.initial_localization_waypoint = self._init_waypoint

            self._client.send_goal(self._action_topic, goal)

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