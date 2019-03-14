#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function

import sys

from socket import error

from tornado.ioloop import IOLoop
from tornado.ioloop import PeriodicCallback
from tornado.web import Application

from rclpy.node import Node
from rclpy.qos import qos_profile_default, QoSDurabilityPolicy
from std_msgs.msg import Int32

from rosbridge_server import RosbridgeWebSocket

from rosbridge_library.capabilities.advertise import Advertise
from rosbridge_library.capabilities.publish import Publish
from rosbridge_library.capabilities.subscribe import Subscribe
from rosbridge_library.capabilities.advertise_service import AdvertiseService
from rosbridge_library.capabilities.unadvertise_service import UnadvertiseService
from rosbridge_library.capabilities.call_service import CallService

from std_msgs.msg import Int32

def shutdown_hook():
    IOLoop.instance().stop()


class RosbridgeWebsocketNode(Node):
    def __init__(self):
        super().__init__('rosbridge_websocket')

        ##################################################
        # Parameter handling                             #
        ##################################################
        retry_startup_delay = self.get_parameter('retry_startup_delay').value
        retry_startup_delay = 2.0 if retry_startup_delay is None
                                  else retry_startup_delay  # seconds


        use_compression = self.get_parameter('use_compression').value
        RosbridgeWebSocket.use_compression = False if use_compression is None
                                                   else use_compression

        # get RosbridgeProtocol parameters
        fragment_timeout = self.get_parameter('fragment_timeout').value
        RosbridgeWebSocket.fragment_timeout = RosbridgeWebSocket.fragment_timeout
                                              if fragment_timeout is None
                                              else fragment_timeout

        delay_between_messages = self.get_parameter('delay_between_messages').value
        RosbridgeWebSocket.delay_between_messages = RosbridgeWebSocket.delay_between_messages 
                                                    if delay_between_messages is None
                                                    else delay_between_messages

        max_message_size = self.get_parameter('max_message_size').value
        RosbridgeWebSocket.max_message_size = RosbridgeWebSocket.max_message_size
                                              if max_message_size is None
                                              else max_message_size

        unregister_timeout = self.get_parameter('unregister_timeout').value
        RosbridgeWebSocket.unregister_timeout = RosbridgeWebSocket.unregister_timeout
                                                if unregister_timeout is None
                                                else unregister_timeout

        bson_only_mode = self.get_parameter('retry_startup_delay').value
        bson_only_mode = False if bson_only_mode is None
                               else bson_only_mode

        if RosbridgeWebSocket.max_message_size == "None":
            RosbridgeWebSocket.max_message_size = None

        # SSL options
        certfile = self.get_parameter('certfile').value
        keyfile = self.get_parameter('keyfile').value
        # if authentication should be used
        authenticate = self.get_parameter('authenticate').value
        RosbridgeWebSocket.authenticate = False if authenticate is None else authenticate

        port = self.get_parameter('port').value
        port = 9090 if port is None else port
        
        address = self.get_parameter('address').value
        address = "" if address is None else address

        # Publisher for number of connected clients
        # QoS profile with transient local durability (latched topic in ROS 1).
        client_count_qos_profile = qos_profile_default
        client_count_qos_profile.durability = QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL

        RosbridgeWebSocket.client_count_pub = self.create_publisher(Int32, 'client_count',
                                                                    qos_profile=client_count_qos_profile)
        RosbridgeWebSocket.client_count_pub.publish(Int32(data=0))

        # Get the glob strings and parse them as arrays.
        topics_glob = self.get_parameter('topics_glob').value
        topics_glob = '' if topics_glob is None else topics_glob

        services_glob = self.get_parameter('topics_glob').value
        services_glob = '' if services_glob is None else services_glob

        params_glob = self.get_parameter('params_glob').value
        params_glob = '' if params_glob is None else params_glob

        RosbridgeWebSocket.topics_glob = [
            element.strip().strip("'")
            for element in topics_glob[1:-1].split(',')
            if len(element.strip().strip("'")) > 0]
        RosbridgeWebSocket.services_glob = [
            element.strip().strip("'")
            for element in services_glob[1:-1].split(',')
            if len(element.strip().strip("'")) > 0]
        RosbridgeWebSocket.params_glob = [
            element.strip().strip("'")
            for element in params_glob[1:-1].split(',')
            if len(element.strip().strip("'")) > 0]

        if "--port" in sys.argv:
            idx = sys.argv.index("--port")+1
            if idx < len(sys.argv):
                port = int(sys.argv[idx])
            else:
                print("--port argument provided without a value.")
                sys.exit(-1)

        if "--address" in sys.argv:
            idx = sys.argv.index("--address")+1
            if idx < len(sys.argv):
                address = int(sys.argv[idx])
            else:
                print("--address argument provided without a value.")
                sys.exit(-1)

        if "--retry_startup_delay" in sys.argv:
            idx = sys.argv.index("--retry_startup_delay") + 1
            if idx < len(sys.argv):
                retry_startup_delay = int(sys.argv[idx])
            else:
                print("--retry_startup_delay argument provided without a value.")
                sys.exit(-1)

        if "--fragment_timeout" in sys.argv:
            idx = sys.argv.index("--fragment_timeout") + 1
            if idx < len(sys.argv):
                RosbridgeWebSocket.fragment_timeout = int(sys.argv[idx])
            else:
                print("--fragment_timeout argument provided without a value.")
                sys.exit(-1)

        if "--delay_between_messages" in sys.argv:
            idx = sys.argv.index("--delay_between_messages") + 1
            if idx < len(sys.argv):
                RosbridgeWebSocket.delay_between_messages = float(sys.argv[idx])
            else:
                print("--delay_between_messages argument provided without a value.")
                sys.exit(-1)

        if "--max_message_size" in sys.argv:
            idx = sys.argv.index("--max_message_size") + 1
            if idx < len(sys.argv):
                value = sys.argv[idx]
                if value == "None":
                    RosbridgeWebSocket.max_message_size = None
                else:
                    RosbridgeWebSocket.max_message_size = int(value)
            else:
                print("--max_message_size argument provided without a value. (can be None or <Integer>)")
                sys.exit(-1)

        if "--unregister_timeout" in sys.argv:
            idx = sys.argv.index("--unregister_timeout") + 1
            if idx < len(sys.argv):
                unregister_timeout = float(sys.argv[idx])
            else:
                print("--unregister_timeout argument provided without a value.")
                sys.exit(-1)

        if "--topics_glob" in sys.argv:
            idx = sys.argv.index("--topics_glob") + 1
            if idx < len(sys.argv):
                value = sys.argv[idx]
                if value == "None":
                    RosbridgeWebSocket.topics_glob = []
                else:
                    RosbridgeWebSocket.topics_glob = [element.strip().strip("'") for element in value[1:-1].split(',')]
            else:
                print("--topics_glob argument provided without a value. (can be None or a list)")
                sys.exit(-1)

        if "--services_glob" in sys.argv:
            idx = sys.argv.index("--services_glob") + 1
            if idx < len(sys.argv):
                value = sys.argv[idx]
                if value == "None":
                    RosbridgeWebSocket.services_glob = []
                else:
                    RosbridgeWebSocket.services_glob = [element.strip().strip("'") for element in value[1:-1].split(',')]
            else:
                print("--services_glob argument provided without a value. (can be None or a list)")
                sys.exit(-1)

        if "--params_glob" in sys.argv:
            idx = sys.argv.index("--params_glob") + 1
            if idx < len(sys.argv):
                value = sys.argv[idx]
                if value == "None":
                    RosbridgeWebSocket.params_glob = []
                else:
                    RosbridgeWebSocket.params_glob = [element.strip().strip("'") for element in value[1:-1].split(',')]
            else:
                print("--params_glob argument provided without a value. (can be None or a list)")
                sys.exit(-1)

        if ("--bson_only_mode" in sys.argv) or bson_only_mode:
            RosbridgeWebSocket.bson_only_mode = bson_only_mode

        # To be able to access the list of topics and services, you must be able to access the rosapi services.
        if RosbridgeWebSocket.services_glob:
            RosbridgeWebSocket.services_glob.append("/rosapi/*")

        Subscribe.topics_glob = RosbridgeWebSocket.topics_glob
        Advertise.topics_glob = RosbridgeWebSocket.topics_glob
        Publish.topics_glob = RosbridgeWebSocket.topics_glob
        AdvertiseService.services_glob = RosbridgeWebSocket.services_glob
        UnadvertiseService.services_glob = RosbridgeWebSocket.services_glob
        CallService.services_glob = RosbridgeWebSocket.services_glob

        ##################################################
        # Done with parameter handling                   #
        ##################################################

        application = Application([(r"/", RosbridgeWebSocket), (r"", RosbridgeWebSocket)])

        connected = False
        while not connected and not rospy.is_shutdown():
            try:
                if certfile is not None and keyfile is not None:
                    application.listen(port, address, ssl_options={ "certfile": certfile, "keyfile": keyfile})
                else:
                    application.listen(port, address)
                rospy.loginfo("Rosbridge WebSocket server started on port %d", port)
                connected = True
            except error as e:
                rospy.logwarn("Unable to start server: " + str(e) +
                              " Retrying in " + str(retry_startup_delay) + "s.")
                rospy.sleep(retry_startup_delay)

        IOLoop.instance().start()


def main(args=None):
    if args is None:
        args = sys.argv
    
    rclpy.init(args=args)
    rosbridge_websocket_node = RosbridgeWebsocketNode(args)
    rclpy.spin(rosbridge_websocket_node)

    node.destroy_node()
    rclpy.shutdown()
    shutdown_hook()     # shutdown hook to stop the server

if __name__ == '__main__':
    main()
