# -*- coding: utf-8 -*-
from http.server import BaseHTTPRequestHandler,HTTPServer
import socket
import socketserver
import cv2
import numpy as np
from PIL import ImageFont, ImageDraw, Image
import datetime as dt
from queue import Empty
from io import BytesIO
import logging
import traceback
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import threading
import sys



class StreamingHandler(BaseHTTPRequestHandler):
    PAGE="""\
        <html>
        <head>
        <title>Rover Video View Stream</title>
        </head>
        <body>
        <img src="stream.mjpg" width="640" height="480" />

        </body>
        </html>
        """
    output = None

    def __init__(self, shared_object, *args):
        self.shared_object = shared_object
        super().__init__(*args)
    
    def do_GET(self):
        self.resolution = (1024, 768) # FIXME: remove hardcodes and use args instead
        self.lock = threading.Lock()
        self.image = None
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/index.html':
            content = self.PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            while True:
                image = self.shared_object.get_image()
                if image:
                    frame = image.getvalue()
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
        else:
            self.send_error(404)
            self.end_headers()

      

class StreamingServer(socketserver.ThreadingMixIn, HTTPServer):
    allow_reuse_address = True
    daemon_threads = True

    # def __init__(self,  shared, *args):
        
        # super().__init__(self, *args)

class SharedObject:
    def __init__(self, buf, lock):
        self.buf = buf
        self.lock = lock

    def set_image(self, image, timeout = 0.01):
        # lock = self.lock.acquire(timeout = timeout)
        print("Writing image")
        # if lock:
        self.buf = image
        # self.lock.release()

    def get_image(self, timeout = 0.01):
        # lock = self.lock.acquire(timeout = timeout)
        # if lock:
        # self.buf.truncate()
        image =  self.buf
            # self.lock.release()
        # else:
            # image = None
        return image


               

class VideoStream(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.get_logger().info('Started')
        self.subscription = self.create_subscription(
            CompressedImage,
            'raw_images',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.address = ('', 1025)
        self.shared_object = SharedObject(BytesIO(), threading.Lock())

        def handler(*args):
            StreamingHandler(self.shared_object, *args)

        self.server = StreamingServer(self.address, handler)
        self.start()
    
    def start(self):
        self.get_logger().warning("Serving at http://" + socket.getfqdn() + ":" + str(self.address[1]) + "/stream.mjpg")
        thread = threading.Thread(target=self.server.serve_forever)
        thread.dameon = True

        try: 
            thread.start()
        except KeyboardInterrupt:
            self.server.shutdown()
            sys.exit(0)

    def listener_callback(self, msg):
        print('listener called')
        self.shared_object.set_image(BytesIO(msg.data))

def main():
    print('Starting Video Stream')
    rclpy.init()
    video = VideoStream()
    rclpy.spin(video)
    video.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()