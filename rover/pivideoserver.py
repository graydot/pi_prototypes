# -*- coding: utf-8 -*-
from http.server import BaseHTTPRequestHandler,HTTPServer
import socket
import socketserver
import cv2
import numpy as np
from PIL import ImageFont, ImageDraw, Image
import datetime as dt
from io import BytesIO
import logging
import traceback



class StreamingHandler(BaseHTTPRequestHandler):
    output = None
        
    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/index.html':
            content = PAGE.encode('utf-8')
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
            try:
                while True:
                    with self.output.condition:
                        self.output.condition.wait()
                        frame = self.output.frame
                        # now add timestamp to jpeg
                        # Convert to PIL Image
                        cv2.CV_LOAD_IMAGE_COLOR = 1 # set flag to 1 to give colour image
                        npframe = np.fromstring(frame, dtype=np.uint8)
                        pil_frame = cv2.imdecode(npframe,cv2.CV_LOAD_IMAGE_COLOR)
                        #pil_frame = cv2.imdecode(frame,-1)
                        cv2_im_rgb = cv2.cvtColor(pil_frame, cv2.COLOR_BGR2RGB)
                        pil_im = Image.fromarray(cv2_im_rgb)

                        draw = ImageDraw.Draw(pil_im)

                        # Choose a font
                        font = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeSans.ttf", 25)
                        myText = "Rover Eyes "+ dt.datetime.now().strftime('%Y-%m-%d %H:%M:%S')

                        # Draw the text
                        color = 'rgb(255,255,255)'
                        #draw.text((0, 0), myText,fill = color, font=font)

                        # get text size
                        text_size = font.getsize(myText)

                        # set button size + 10px margins
                        button_size = (text_size[0]+20, text_size[1]+10)

                        # create image with correct size and black background
                        button_img = Image.new('RGBA', button_size, "black")

                        #button_img.putalpha(128)
                        # put text on button with 10px margins
                        button_draw = ImageDraw.Draw(button_img)
                        button_draw.text((10, 5), myText, fill = color, font=font)

                        # put button on source image in position (0, 0)

                        pil_im.paste(button_img, (0, 0))
                        bg_w, bg_h = pil_im.size
                        # WeatherSTEM logo in lower left
                        size = 64
                        # WSLimg = Image.open("WeatherSTEMLogoSkyBackground.png")
                        # WSLimg.thumbnail((size,size),Image.ANTIALIAS)
                        # pil_im.paste(WSLimg, (0, bg_h-size))

                        # SkyWeather log in lower right
                        # SWLimg = Image.open("SkyWeatherLogoSymbol.png")
                        # SWLimg.thumbnail((size,size),Image.ANTIALIAS)
                        # pil_im.paste(SWLimg, (bg_w-size, bg_h-size))

                        # Save the image
                        buf= BytesIO()
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
    
    def __init__(self, address, handler, output):
        HTTPServer.__init__(self, output)
        socketserver.ThreadingMixIn.__init__(self)
        
class PiVideoServer:
    def __init__(self, output):
        self.output = output
        self.address = ('', 1025)
        StreamingHandler.output = output
        self.server = StreamingServer(self.address, StreamingHandler)
    
    def start_server(self):
        logging.warning("Serving at http://" + socket.getfqdn() + ":" + str(self.address[1]))
        self.server.serve_forever()
