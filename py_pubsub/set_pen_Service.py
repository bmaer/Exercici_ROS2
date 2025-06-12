import rclpy
from rclpy.node import Node
from turtlesim.srv import SetPen

class set_pen_Service(Node):
    def __init__(self):
        super().__init__('set_pen_Service')
        self.srv = self.create_service(SetPen, 'set_pen_service', self.set_pen_callback)

    def set_pen_callback(self, request, response):
        response.r = request.r
        response.g = request.g
        response.b = request.b
        response.off = request.off
        return response

    def main(args=None):
        rclpy.init(args=args)

        set_pen_Service = set_pen_Service()

        rclpy.spin(set_pen_Service)

        rclpy.shutdown()

    if __name__ == '__main__':
        main()