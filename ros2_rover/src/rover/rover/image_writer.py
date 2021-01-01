from io import BytesIO

class ImageWriter:
    def __init__(self, publisher):
        self.publisher_ = publisher
        self.stream = BytesIO
    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # New frame, put the last frame's data in the queue
            size = self.stream.tell()
            if size:
                self.stream.seek(0)
                self.publisher_.publish("got it")  #(self.stream.getvalue())
                self.stream.seek(0)
        self.stream.write(buf)