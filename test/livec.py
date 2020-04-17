#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int32
import argparse
import sys


parser = argparse.ArgumentParser(
	description='点群出力テスト〜設定閾値以下の点群数の場合exitする',
	epilog='',
	add_help=True,
)

parser.add_argument('pcth', metavar='pc_threshold', help='点群数閾値',default=1000)    
args = parser.parse_args()

rospy.init_node('test',anonymous=True)

while 1:
	pub_capt=rospy.Publisher('/rovi/X1',Bool,queue_size=1)
	rospy.sleep(1)
	pub_capt.publish(Bool(data=True))
	pc=rospy.wait_for_message('/rovi/pcount',Int32,timeout=5)
	print "pc_count=",pc.data
	if pc.data<int(args.pcth):
		sys.exit()
	rospy.sleep(1)

rospy.is_shutdown()
