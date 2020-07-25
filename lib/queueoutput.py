# -*- coding: utf-8 -*-
from io import BytesIO
class QueueOutput(object):
    def __init__(self, queues, events):
        self.queues = queues
        self.events = events
        self.stream = BytesIO()

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # New frame, put the last frame's data in the queue
            size = self.stream.tell()
            if size:
                self.stream.seek(0)
                for queue in self.queues:
                    queue.put(self.stream.getvalue())
                self.stream.seek(0)
        self.stream.write(buf)

    def flush(self):
        for queue in self.queues:
            queue.close()
            queue.join_thread()
        for finished in self.events:
            finished.set()
