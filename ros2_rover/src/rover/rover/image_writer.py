from io import BytesIO
from sensor_msgs.msg import CompressedImage
from datetime import datetime

class ImageWriter:
    def __init__(self, node, publisher):
        self.node_ = node
        self.publisher_ = publisher
        self.stream = BytesIO()
    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # New frame, put the last frame's data in the queue
            size = self.stream.tell()
            if size:
                self.stream.truncate()  
                self.stream.seek(0)
                image_bytes = self.stream.getvalue()
                print("Wrote at %s" % datetime.now())
                msg = CompressedImage()
                
                # UGLY, find another way to get date
                msg.header.stamp = self.node_.get_clock().now().to_msg()
                msg.format = "jpeg"
                msg.data = image_bytes
                self.publisher_.publish(msg)
                self.stream.seek(0)
        self.stream.write(buf)