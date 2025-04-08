# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) #create_publisher(<type>, <nom_topic>, <buffer>)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.listener_callback, 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.x = 0.0
        self.y = 0.0

    def timer_callback(self):
        msg1 = Twist()

        msg1.linear.x = self.x
        msg1.linear.y = self.y
        msg1.linear.z = 0.0
        self.publisher_.publish(msg1)

        #self.get_logger().info('Publishing: "%s"' % msg1.data)
        

    def listener_callback(self, msg):

        if(msg.x <= 8.0 and msg.x != 9.0):
            #Si la pos en x es menor que 9 segueix en msg1.linear.x = 1.0
            self.x = 1.0
        if(msg.x >= 10.0 and msg.x != 9.0):
            #si no que sigui -1 i es repeteix per la cordenada y
            self.x = -1.0
        if(msg.x > 8.0 and msg.x < 9.0):
            self.x = 0.1
        if(msg.x < 10.0 and msg.x >9.0):
            self.x = -0.1
        if(msg.x == 9.0):
            self.x = 0.0


        if(msg.y <= 7.0 and msg.y != 8.0):
            # Si la pos en y es menor que 8 segueix en msg1.linear.x = 1.0
            self.y = 1.0
        if (msg.y >= 9.0 and msg.y != 8.0):
            # si no que sigui -1 i es repeteix per la cordenada y
            self.y = -1.0
        if(msg.y > 7.0 and msg.y < 8.0):
            self.y = 0.1
        if (msg.y < 9.0 and msg.y > 8.0):
            self.y = -0.1
        if(msg.y == 8.0):
            self.y = 0.0


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
