#! /usr/bin/env python3

import argparse
import os
import re
import sqlite3
import string
import sys
import textwrap

from lxml import etree

from sros2.api import NodeName
from sros2.api import TopicInfo
from sros2.policy import (
    dump_policy,
    load_policy,
    POLICY_VERSION,
)


# fuctions copied from sros2 for now
def get_policy(policy_file_path):
    if os.path.isfile(policy_file_path):
        return load_policy(policy_file_path)
    else:
        profiles = etree.Element('profiles')
        policy = etree.Element('policy')
        policy.attrib['version'] = POLICY_VERSION
        policy.append(profiles)
        return policy


def get_profile(policy, node_name):
    profile = policy.find(
        path='profiles/profile[@ns="{ns}"][@node="{node}"]'.format(
            ns=node_name.ns,
            node=node_name.node))
    if profile is None:
        profile = etree.Element('profile')
        profile.attrib['ns'] = node_name.ns
        profile.attrib['node'] = node_name.node
        profiles = policy.find('profiles')
        profiles.append(profile)
    return profile


def get_permissions(profile, permission_type, rule_type, rule_qualifier):
    permissions = profile.find(
        path='{permission_type}s[@{rule_type}="{rule_qualifier}"]'.format(
            permission_type=permission_type,
            rule_type=rule_type,
            rule_qualifier=rule_qualifier))
    if permissions is None:
        permissions = etree.Element(permission_type + 's')
        permissions.attrib[rule_type] = rule_qualifier
        profile.append(permissions)
    return permissions


# Custom functions for conversion
def get_node_permission_dict_from_table(
        database, permission_type, permission_equivalence, dict_to_update):
    with database:
        cur = database.cursor()
        cur.execute(
            'SELECT ParticipantData_user_data, %s FROM %s' % (
                permission_equivalence[permission_type]['column_name'],
                permission_equivalence[permission_type]['table_name'])
        )
        all_pub_list = cur.fetchall()
    pub_dict = {}
    for tup in all_pub_list:
        user_data = tup[0]
        dds_topic = tup[1]
        if user_data not in pub_dict.keys():
            pub_dict[user_data] = [TopicInfo(fqn=dds_topic, type=None)]
        else:
            if dds_topic not in pub_dict[user_data]:
                pub_dict[user_data].append(TopicInfo(fqn=dds_topic, type=None))

    matching_template = string.Template(r'(;|^)$user_data_field=(.*?);')

    for key, value in pub_dict.items():
        if key is None:
            continue

        decoded_key = key.decode()
        node_name_match = re.search(
            matching_template.substitute(user_data_field='name'), decoded_key)
        node_namespace_match = re.search(
            matching_template.substitute(user_data_field='namespace'), decoded_key)
        if node_name_match.groups()[1] is None or node_namespace_match.groups()[1] is None:
            assert False, key + ': couldnt find pattern'
        node_name = node_name_match.groups()[1]
        node_namespace = node_namespace_match.groups()[1]
        fqn = node_namespace + node_name
        node_name_tuple = NodeName(
            node=node_name,
            ns=node_namespace,
            fqn=fqn
        )
        if fqn not in dict_to_update.keys():
            dict_to_update[fqn] = {'node_name': node_name_tuple, permission_type: value}
        else:
            if permission_type not in dict_to_update[fqn].keys():
                dict_to_update[fqn][permission_type] = value
            else:
                if value not in dict_to_update[fqn][permission_type]:
                    dict_to_update[fqn][permission_type].append(value, type=None)


def add_topics_permissions(profile, topics, permission_type, node_name):
    permissions = get_permissions(profile, 'topic', permission_type, 'ALLOW')
    for topic in topics:
        permission = etree.Element('topic')
        permission.text = topic.fqn
        permissions.append(permission)


def policy_from_dict(ros_nodes_dict, permission_equivalence, policy_file_in=''):
    policy = get_policy(policy_file_in)
    node_names = [ros_nodes_dict[x]['node_name'] for x in ros_nodes_dict.keys()]

    if not len(node_names):
        print('Dict is empty.',
              file=sys.stderr)
        return 1

    for node in ros_nodes_dict.keys():
        profile = get_profile(policy, ros_nodes_dict[node]['node_name'])
        for permission_type in permission_equivalence.keys():
            if permission_type in ros_nodes_dict[node].keys():
                add_topics_permissions(
                    profile, ros_nodes_dict[node][permission_type],
                    permission_type, ros_nodes_dict[node]['node_name'])
    return policy


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '-i', '--input-db', required=True,
        help='SQLite database with discovery data')
    parser.add_argument(
        '-p', '--policy-file',
        help='policy file to overwrite')
    args = parser.parse_args(argv)

    ros_node_dict = {}

    dds_db = sqlite3.connect(args.input_db)

    create_view_template = string.Template(textwrap.dedent("""
            CREATE VIEW `Node_to_${connexion_type}` AS SELECT DISTINCT
                DCPSParticipant.ParticipantData_key,
                DCPSParticipant.ParticipantData_user_data,
                DCPS${connexion_type}.${connexion_type}Data_topic_name
            FROM DCPSParticipant
            INNER JOIN DCPS${connexion_type}
                ON DCPSParticipant.ParticipantData_key=DCPS${connexion_type}.${connexion_type}Data_participant_key AND DCPSParticipant.ParticipantData_user_data IS NOT NULL"""))
    with dds_db:
        cur = dds_db.cursor()
        cur.execute(create_view_template.substitute(connexion_type='Publication'))
        cur.execute(create_view_template.substitute(connexion_type='Subscription'))

    permission_equivalence = {
        'publish': {
            'table_name': 'Node_to_Publication',
            'column_name': 'PublicationData_topic_name'},
        'subscribe': {
            'table_name': 'Node_to_Subscription',
            'column_name': 'SubscriptionData_topic_name'},
    }

    get_node_permission_dict_from_table(dds_db, 'publish', permission_equivalence, ros_node_dict)
    get_node_permission_dict_from_table(dds_db, 'subscribe', permission_equivalence, ros_node_dict)

    policy = policy_from_dict(ros_node_dict, permission_equivalence)

    if args.policy_file is not None:
        with open(args.policy_file, 'w') as stream:
            dump_policy(policy, stream)
    else:
        print(etree.tostring(policy, pretty_print=True).decode())


if __name__ == '__main__':
    main()
