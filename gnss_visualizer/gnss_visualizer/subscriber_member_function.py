import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from sensor_msgs.msg import NavSatFix    # CHANGE
from ublox_msgs.msg import NavRELPOSNED    # CHANGE

import math
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import requests
from io import BytesIO
from PIL import Image

longitude = 127.362
latitude = 36.374
lat = math.pi * latitude / 180
size = 7
level = 19
tile_res = 256

position = [0,0]
direction = [0,0]

max_size = pow(2, level)

center_x = math.ceil(((longitude + 180.0) / 360.0) * pow(2.0, level))
center_y = math.ceil((1.0 - math.log(math.tan(lat) + 1.0 / math.cos(lat)) / math.pi) / 2.0 * pow(2.0, level))

top = int(max(0, center_y - size / 2))
left = int(max(0, center_x - size / 2))
right = int(min(max_size, left + size))
bottom = int(min(max_size, top + size))
print(top, left, right, bottom)

result = Image.new('RGB', (tile_res * size, tile_res*size))
for y in range(top, bottom):
    for x in range(left, right):
        url = "http://localhost:8080/wmts/gm_layer/gm_grid/{}/{}/{}.png".format(level, x, y)
        response = requests.get(url)
        img = Image.open(BytesIO(response.content))
        result.paste(im=img, box=(tile_res*(x - left), tile_res*(y -top)))

plt.ion()
plt.imshow(result)
x = round(((longitude + 180.0) / 360.0) * pow(2.0, level) * tile_res) - left * tile_res
y = round((1.0 - math.log(math.tan(lat) + 1.0 / math.cos(lat)) / math.pi) / 2.0 * pow(2.0, level) * tile_res) - top * tile_res
plt.draw()
plt.pause(0.001)

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            NavRELPOSNED,
            '/rover/navrelposned',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.subscription1 = self.create_subscription(
            NavSatFix,
            '/base/fix',
            self.listener_callback1,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        direction[0] = msg.rel_pos_e
        direction[1] = msg.rel_pos_n

    def listener_callback1(self, msg):
        x = round(((msg.longitude + 180.0) / 360.0) * pow(2.0, level) * tile_res) - left * tile_res
        lat = math.pi * msg.latitude / 180
        y = round((1.0 - math.log(math.tan(lat) + 1.0 / math.cos(lat)) / math.pi) / 2.0 * pow(2.0, level) * tile_res) - top * tile_res
        position[0] = x
        position[1] = y
        plt.quiver(x,y, direction[0],direction[1], scale = 2000, headwidth = 1, color = 'r')
        plt.draw()
        plt.pause(0.001)
        
        print(x)
        print(y)
        print(msg.latitude)
        print(msg.longitude)
        print(direction[0])
        print(direction[1])


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
