#!/usr/bin/env python

import rclpy
from rclpy.node import Node


from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller

from std_srvs.srv import Trigger

from spot_msgs.srv import CaptureImage


class CaptureImage(EventState):
        '''
        This state can be used for capturing and saving an image locally.

        -- source               string          name of image source (or camera)  
        -- image_save_path      string          path where the image should be saved locally

        #> None
        ># None

        <= continue             indicates successful completion of the state   
        <= failed               indicates failure to complete the execution of the state

        '''

        def __init__(self, spot_name, source, image_save_path):
                # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
                super().__init__(outcomes = ['continue', 'failed'])

                ProxyServiceCaller.initialize(CaptureImage._node)

                self._service_topic = '/' + spot_name + '/capture_image'
                self._service = ProxyServiceCaller({self._service_topic: CaptureImage})
                self._source = source
                self._image_save_path = image_save_path


        def execute(self, userdata):
                # This method is called periodically while the state is active.
                # Main purpose is to check state conditions and trigger a corresponding outcome.
                # If no outcome is returned, the state will stay active.

                request = CaptureImage.Request()
                request.source = self._source
                request.image_save_path = self._image_save_path

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