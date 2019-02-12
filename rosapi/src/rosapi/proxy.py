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

import fnmatch
import socket

from ros2node.api import get_node_names, get_publisher_info, get_service_info, get_subscriber_info
from ros2service.api import get_service_names, get_service_names_and_types
from ros2topic.api import get_topic_names, get_topic_names_and_types

# from rosservice import get_service_list
# from rosservice import get_service_type as rosservice_get_service_type
# from rosservice import get_service_node as rosservice_get_service_node
# from rosservice import get_service_uri
# from rosservice import rosservice_find
# from rostopic import find_by_type
# from rostopic import get_topic_type as rosservice_get_topic_type
# from ros import rosnode, rosgraph
# from rosnode import get_node_names
# from rosgraph.masterapi import Master

from rosapi.msg import TypeDef

from .glob_helper import filter_globs, any_match

_node = None

def init(node):
    """
    Initializes proxy module with a rclpy.node.Node for further use.
    This function has to be called before any other for the module to work.
    """
    global _node
    _node = node

def get_topics(topics_glob, include_hidden=False):
    """ Returns a list of all the active topics in the ROS system """
    try:
        topic_names = get_topic_names(node=_node, include_hidden_topics=include_hidden)
        return filter_globs(topics_glob, topic_names)
    except:
        return []


def get_topics_types(topics, topics_glob):
    try:
        types = []
        for i in topics:
            types.append(get_topic_type(i, topics_glob))
        return types
    except:
        return[]


def get_topics_and_types(topics_glob, include_hidden=False):
    try:
        topic_names_and_types = get_topic_names_and_types(node=_node, include_hidden_topics=include_hidden)
        # topic[0] has the topic name and topic[1] has the type wrapped in a list.
        all_topics = set([topic[0] for topic in topic_names_and_types])
        filtered_topics = set(filter_globs(topics_glob, all_topics))
        filtered_topic_types = [topic[1][0] for topic in topic_names_and_types if topic[0] in filtered_topics]
        return filtered_topics, filtered_topic_types
    except:
        return [], []


def get_topics_for_type(topic_type, topics_glob, include_hidden=False):
    try:
        topic_names_and_types = get_topic_names_and_types(node=_node, include_hidden_topics=include_hidden)
        # topic[0] has the topic name and topic[1] has the type wrapped in a list.
        topics_for_type = [topic[0] for topic in topic_names_and_types if topic[1][0] == topic_type]
        return filter_globs(topics_glob, topics_for_type)
    except:
        return []


def get_services(services_glob, include_hidden=False):
    """ Returns a list of all the services advertised in the ROS system """
    # Filter the list of services by whether they are public before returning.
    try:
        service_names = get_service_names(node=_node, include_hidden_services=include_hidden)
        return filter_globs(services_glob, service_names)
    except:
        return []


def get_services_for_type(service_type, services_glob, include_hidden=False):
    """ Returns a list of services as specific service type """
    # Filter the list of services by whether they are public before returning.
    try:
        services_names_and_types = get_service_names_and_types(node=_node, include_hidden_services=include_hidden)
        # service[0] has the topic name and service[1] has the type wrapped in a list.
        services_for_type = [service[0] for service in services_names_and_types if service[1][0] == service_type]
        return filter_globs(services_glob, services_for_type)
    except:
        return []


def get_nodes(include_hidden=False):
    """ Returns a list of all the nodes registered in the ROS system """
    try:
        node_names = get_node_names(node=_node, include_hidden_nodes=include_hidden)
        # Discard the node that requests the names.
        full_names = [node_name.full_name for node_name in node_names if node_name.full_name != '/requester_rosapi_Nodes']
        return full_names
    except:
        return []

def get_node_info(node_name, include_hidden=False):
    node_names = get_node_names(node=_node, include_hidden_nodes=include_hidden)
    if node_name in [n.full_name for n in node_names]:
        # Only the name of each item is required as output.
        subscribers = get_subscriber_info(node=_node, remote_node_name=node_name)
        subscribers = [subscriber.name for subscriber in subscribers]

        publishers = get_publisher_info(node=_node, remote_node_name=node_name)
        publishers = [publisher.name for publisher in publishers]

        services = get_service_info(node=_node, remote_node_name=node_name)
        services = [service.name for service in services]

        return subscribers, publishers, services

def get_node_publications(node_name):
    """ Returns a list of topic names that are been published by the specified node """
    try:
        publisher_info = get_publisher_info(node=_node, remote_node_name=node_name)
        return publisher_info
    except:
        return []


def get_node_subscriptions(node_name):
    """ Returns a list of topic names that are been subscribed by the specified node """
    try:
        subscriber_info = get_subscriber_info(node=_node, remote_node_name=node_name)
        return subscriber_info
    except:
        return []


def get_node_services(node_name):
    """ Returns a list of service names that are been hosted by the specified node """
    try:
        service_info = get_service_info(node=_node, remote_node_name=node_name)
        return service_info
    except:
        return []


def get_topic_type(topic, topics_glob):
    """ Returns the type of the specified ROS topic """
    # Check if the topic is hidden or public.
    # If all topics are public then the type is returned
    if any_match(str(topic), topics_glob):
        # If the topic is published, return its type
        topic_type, _, _ = rosservice_get_topic_type(topic)
        if topic_type is None:
            # Topic isn't published so return an empty string
            return ""
        return topic_type
    else:
        # Topic is hidden so return an empty string
        return ""


def filter_action_servers(topics):
    """ Returns a list of action servers """
    # Note(@jubeira): filtering by topic should be enough; services can be taken into account as well.
    action_servers = []
    possible_action_server = ''
    possibility = [0, 0]

    action_topics = ['feedback', 'status']
    for topic in sorted(topics):
        split = topic.split('/')
        if(len(split) >= 4):
            topic = split.pop()
            action_prefix = split.pop()
            if action_prefix != '_action':
                continue

            namespace = '/'.join(split)
            if(possible_action_server != namespace):
                possible_action_server = namespace
                possibility = [0, 0]
            if possible_action_server == namespace and topic in action_topics:
                _node.get_logger().info('Possibility 1 for topic: {} namespace: {}'.format(topic, namespace))
                possibility[action_topics.index(topic)] = 1
            if all(p == 1 for p in possibility):
                action_servers.append(possible_action_server)
                possibility = [0, 0]

    return action_servers


def get_service_type(service, services_glob):
    """ Returns the type of the specified ROS service, """
    # Check if the service is hidden or public.
    if any_match(str(service), services_glob):
        try:
            return rosservice_get_service_type(service)
        except:
            return ""
    else:
        # Service is hidden so return an empty string.
        return ""


def get_publishers(topic, topics_glob):
    """ Returns a list of node names that are publishing the specified topic """
    try:
        if any_match(str(topic), topics_glob):
            publishers, subscribers, services = Master('/rosbridge').getSystemState()
            pubdict = dict(publishers)
            if topic in pubdict:
                return pubdict[topic]
            else:
                return []
        else:
            return []
    except socket.error:
        return []


def get_subscribers(topic, topics_glob):
    """ Returns a list of node names that are subscribing to the specified topic """
    try:
        if any_match(str(topic), topics_glob):
            publishers, subscribers, services = Master('/rosbridge').getSystemState()
            subdict = dict(subscribers)
            if topic in subdict:
                return subdict[topic]
            else:
                return []
        else:
            return []
    except socket.error:
        return []


def get_service_providers(queried_type, services_glob):
    """ Returns a list of node names that are advertising a service with the specified type """
    _, _, services = Master('/rosbridge').getSystemState()

    service_type_providers = []
    for service, providers in services:
        service_type = get_service_type(service, services_glob)

        if service_type == queried_type:
            service_type_providers += providers
    return service_type_providers


def get_service_node(service):
    """ Returns the name of the node that is providing the given service, or empty string """
    node = rosservice_get_service_node(service)
    if node == None:
        node = ""
    return node


def get_service_host(service):
    """ Returns the name of the machine that is hosting the given service, or empty string """
    uri = get_service_uri(service)
    if uri == None:
        uri = ""
    else:
        uri = uri[9:]
        uri = uri[:uri.find(':')]
    return uri
