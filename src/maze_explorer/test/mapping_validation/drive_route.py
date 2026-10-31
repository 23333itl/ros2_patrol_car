#!/usr/bin/env python3
"""Simulator-only motion/revisit test. Launch with explore:=false. No truth input."""
import json, math, signal, sys, time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from tf2_ros import Buffer, TransformListener
rclpy.init()
node=Node('mapping_test_route',parameter_overrides=[Parameter('use_sim_time',value=True)])
client=ActionClient(node,NavigateToPose,'navigate_to_pose');buffer=Buffer();listener=TransformListener(buffer,node)
publisher=node.create_publisher(Twist,'/cmd_vel',10)
processed=[];events=[];phases=[];active=None;stopping=False
subscription=node.create_subscription(PoseWithCovarianceStamped,'/pose',lambda m:processed.append(m.header.stamp.sec+m.header.stamp.nanosec*1e-9),100)
output=Path(sys.argv[1])
def stop(*_):
 global stopping
 stopping=True
signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
def wait(future,seconds=150):
 deadline=time.monotonic()+seconds
 while not future.done() and time.monotonic()<deadline and not stopping:rclpy.spin_once(node,timeout_sec=.02)
 if not future.done():raise RuntimeError('Route stopped or timed out')
 return future.result()
def phase(name,linear,angular,duration):
 start=node.get_clock().now().nanoseconds*1e-9;deadline=time.monotonic()+duration*3+15
 msg=Twist();msg.linear.x=linear;msg.angular.z=angular
 while not stopping and node.get_clock().now().nanoseconds*1e-9-start<duration:
  if time.monotonic()>deadline:raise RuntimeError('Motion phase clock timeout')
  publisher.publish(msg);rclpy.spin_once(node,timeout_sec=.025)
 publisher.publish(Twist());end=node.get_clock().now().nanoseconds*1e-9
 phases.append(dict(name=name,start_sim_s=start,end_sim_s=end,processed_scan_stamps=[t for t in processed if start<=t<=end]))
 output.with_name('motion_phases.json').write_text(json.dumps(phases,indent=2))
try:
 if not client.wait_for_server(timeout_sec=90):raise RuntimeError('Nav2 not ready')
 for name in ('controller_server','planner_server','bt_navigator','velocity_smoother'):
  state=node.create_client(GetState,'/'+name+'/get_state')
  if not state.wait_for_service(timeout_sec=60):raise RuntimeError('Missing lifecycle '+name)
  deadline=time.monotonic()+60
  while time.monotonic()<deadline:
   if wait(state.call_async(GetState.Request()),5).current_state.id==3:break
   for _ in range(10):rclpy.spin_once(node,timeout_sec=.05)
  else:raise RuntimeError('Inactive lifecycle '+name)
 for item in [('stationary',0.,0.,5.),('straight',.08,0.,2.),('stop',0.,0.,2.),('reverse',-.08,0.,2.),('stop',0.,0.,2.),('left',0.,.25,6.8),('stop',0.,0.,2.),('right',0.,-.25,6.8),('stop',0.,0.,3.)]:phase(*item)
 origin=None;deadline=time.monotonic()+30
 while origin is None and time.monotonic()<deadline:
  rclpy.spin_once(node,timeout_sec=.05)
  try:origin=buffer.lookup_transform('map','base_footprint',Time())
  except Exception:pass
 if origin is None:raise RuntimeError('No initial map pose')
 q=origin.transform.rotation;yaw=math.atan2(2*q.w*q.z,1-2*q.z*q.z)
 route=[(.8,0),(.8,.8),(0,.8)]+[(0,.8*i) for i in range(2,8)]+[(.8,5.6),(0,5.6)]+[(0,.8*i) for i in range(6,0,-1)]+[(.8,.8),(.8,0),(0,0)]
 for repeat in range(2):
  for index,(x,y) in enumerate(route):
   goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=node.get_clock().now().to_msg()
   goal.pose.pose.position.x=origin.transform.translation.x+math.cos(yaw)*x-math.sin(yaw)*y
   goal.pose.pose.position.y=origin.transform.translation.y+math.sin(yaw)*x+math.cos(yaw)*y;goal.pose.pose.orientation.w=1.
   active=wait(client.send_goal_async(goal))
   if not active.accepted:raise RuntimeError('Route goal rejected')
   result=wait(active.get_result_async());active=None
   event=dict(repeat=repeat,waypoint=index,status=result.status,sim_s=node.get_clock().now().nanoseconds*1e-9)
   events.append(event);print(json.dumps(event),flush=True)
   if result.status!=4:raise RuntimeError('Route navigation failed')
except RuntimeError as error:print(str(error),flush=True)
finally:
 publisher.publish(Twist())
 if active:
  try:stopping=False;wait(active.cancel_goal_async(),5)
  except Exception:pass
 output.write_text(json.dumps(events,indent=2));node.destroy_node();rclpy.shutdown()
