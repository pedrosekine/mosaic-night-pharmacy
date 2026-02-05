
import rclpy
from moveit.planning import MoveItPy
rclpy.init()
try:
    m = MoveItPy(node_name='test_inspector')
    print('DIR:', dir(m))
except Exception as e:
    print(e)
rclpy.shutdown()

