import socket
import time
import sys
from datetime import datetime
import struct
sys.path.append("./pymsg")
from pymsg.packet_pb2     import Packet
from pymsg.publish_pb2    import Publish
from pymsg.request_pb2    import Request
from pymsg.response_pb2   import Response
from pymsg.pose_pb2       import Pose
from pymsg.subscribe_pb2  import Subscribe
from pymsg.gz_string_pb2  import GzString

MASTER_TCP_IP   = '127.0.0.1'
MASTER_TCP_PORT = 11345
NODE_TCP_IP     = '127.0.0.1'
NODE_TCP_PORT   = 11451
TCP_BUFFER_SIZE = 40960

# Listen for Subscribers
s_sub = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s_sub.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s_sub.bind((NODE_TCP_IP, NODE_TCP_PORT))
s_sub.listen(5)

# Register as a Publisher with Gazebo
pk            = Packet()
pk.stamp.sec  = int(time.time())
pk.stamp.nsec = datetime.now().microsecond
pk.type       = "advertise"

pub           = Publish()
pub.topic     = "/gazebo/default/interceptor/gimbal_yaw_rate_cmd"
pub.msg_type  = GzString.DESCRIPTOR.full_name
pub.host      = NODE_TCP_IP
pub.port      = NODE_TCP_PORT
pk.serialized_data = pub.SerializeToString()

s_reg = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s_reg.connect((MASTER_TCP_IP, MASTER_TCP_PORT))
s_reg.send(hex(pk.ByteSize()).rjust(8).encode('utf-8'))
s_reg.send(pk.SerializeToString())

conn, address = s_sub.accept()
data = conn.recv(TCP_BUFFER_SIZE)

# Decode Incomming Packet
pk_sub = Packet()
pk_sub.ParseFromString(data[8:])
print("Packet:\n", pk_sub)

while 1:   

    # Prepare data to send
    msg_yaw_rate = GzString()
    msg = "-0.05"
    msg_yaw_rate.data = msg

    # Publish Packet to Subscriber
    pk_pub            = Packet()
    pk_pub.stamp.sec  = int(time.time())
    pk_pub.stamp.nsec = datetime.now().microsecond
    pk_pub.type       = GzString.DESCRIPTOR.full_name
    pk_pub.serialized_data = msg_yaw_rate.SerializeToString()

    conn.send(hex(msg_yaw_rate.ByteSize()).rjust(8).encode('utf-8'))
    conn.send(msg_yaw_rate.SerializeToString())

    print("Success")

