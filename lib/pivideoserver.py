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
        
    def do_GET(self):
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
            while not self.finished.wait(0.1):
                try:
                    frame = None
                    while True:
                        frame = self.queue.get(False)
                except Empty:
                    if frame == None:
                        continue 
                    buf= BytesIO()
                    pil_im = Image.frombytes('RGB', self.resolution, frame)
                    pil_im.save(buf, format= 'JPEG')
                    frame = buf.getvalue()
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
                except Exception as e:
                    traceback.print_exc()
                    logging.warning(
                        'Removed streaming client %s: %s',
                        self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()

class StreamingServer(socketserver.ThreadingMixIn, HTTPServer):
    allow_reuse_address = True
    daemon_threads = True
        
class PiVideoServer:
    def __init__(self, queue, finished, resolution):
        self.address = ('', 1025)
        # need to use class method due to the way Streaming server is constructed
        StreamingHandler.queue = queue 
        StreamingHandler.finished = finished
        StreamingHandler.resolution = resolution
        self.server = StreamingServer(self.address, StreamingHandler)
    
    def start(self):
        logging.warning("Serving at http://" + socket.getfqdn() + ":" + str(self.address[1]) + "/stream.mjpg")
        self.server.serve_forever()
