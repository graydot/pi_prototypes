import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from PIL import Image
import numpy as np
from io import BytesIO
from rover.facessd_mobilenet_v2 import FaceSSD_MobileNet_V2_EdgeTPU


class ObjectDetector(Node):

    def __init__(self):
        super().__init__('object_detector')
        self.get_logger().info('Started')
        self.subscription = self.create_subscription(
            CompressedImage,
            'raw_images',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.imagepublisher = self.create_publisher(CompressedImage, 'tagged_images', 10)
        self.targetpublisher = self.create_publisher(CompressedImage, 'target_point', 10)
        
        self.model = FaceSSD_MobileNet_V2_EdgeTPU()
        labels = ['face']
        self.label_idxs = self.model.label_to_category_index(labels)
        

    def listener_callback(self, msg):
        im = Image.open(BytesIO(msg.data))
        tensor_im = pil_im.copy()
        tensor_im = tensor_im.resize((320,320))

        predictions = self.model.predict(np.array(tensor_im))
        boxes = predictions.get('detection_boxes')
        if len(boxes):
            classes = predictions.get('detection_classes')
            scores = predictions.get('detection_scores')
            objects = filter(lambda item: item in self.label_idxs, classes)
            def item_to_hashmap(index_and_item):
                index, item = index_and_item
                return {
                    "i": index,
                    "name" : self.model.category_index[item]['name'],
                    "box" : boxes[index],
                    "score" : scores[index]
                }
            objects = list(map(item_to_hashmap , enumerate(objects)))
            objects = list(filter(lambda item: item["score"] > 0.5, objects))

            if len(objects) > 0:
                tracked_object = objects[0]
            else:
                tracked_object = None
        else:
            tracked_object = None

        # TODO: Consider moving everything below to a separate Node later
        #calculate FPS
        current_time = time.time()
        time_elapsed = current_time - start_time
        if time_elapsed >= 1:
            fps = int(frame_count/time_elapsed)
            start_time = current_time
            frame_count = 0
        frame_count += 1

        draw = ImageDraw.Draw(pil_im)
        (im_width, im_height) = pil_im.size

        for object in objects:
            (y1, x1, y2, x2) = object['box']
            (l, r, t, b) = (x1 * im_width, x2 * im_width, y1 * im_height, y2 * im_height)
            if object == tracked_object:
                thickness = 2
                color = 'red'
            else:
                thickness = 1
                color = 'LawnGreen'
            draw.line([(l, t), (l, b), (r, b), (r, t), (l, t)],\
                width = thickness, fill = color)

            try:
                font = ImageFont.truetype('arial.ttf', 24)
            except IOError:
                font = ImageFont.load_default()

            display_str = object['name'] + ", " + str(object["score"])
            # If the total height of the display strings added to the top of the bounding
            # box exceeds the top of the image, stack the strings below the bounding box
            # instead of above.
            display_str_height = font.getsize(display_str)[1]
            # Each display_str has a top and bottom margin of 0.05x.
            total_display_str_height = (1 + 2 * 0.05) * display_str_height

            if t > total_display_str_height:
                text_bottom = t
            else:
                text_bottom = b + total_display_str_height
            text_width, text_height = font.getsize(display_str)
            margin = np.ceil(0.05 * text_height)
            draw.rectangle(
                [(l, text_bottom - text_height - 2 * margin), (l + text_width,
                                                                    text_bottom)],
                fill=color)
            draw.text(
                (l + margin, text_bottom - text_height - margin),
                display_str,
                fill='black',
                font=font)
            text_bottom -= text_height - 2 * margin

        font = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeSans.ttf", 8)
        myText = "Rover Eyes " + dt.datetime.now().strftime('%Y-%m-%d %H:%M:%S') +\
            " FPS " + str(fps) + \
            (". Tracking " + tracked_object["name"] if tracked_object else "")

        # Draw the text
        # ***************** if tracking change color to something else
        if tracked_object:
            color = 'rgb(255, 0, 0)'
        else:
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

        # Optimization. Run Face Detection only once per runModelPerIterations.
        # But process existing bounding boxes every iteration
        if tracked_object:
            (y1, x1, y2, x2) = tracked_object['box']
            self.center_x.value = (x1 + x2)/2
            self.center_y.value = (y1 + y2)/2


def main(args=None):
    rclpy.init(args=args)

    object_detector = ObjectDetector()

    rclpy.spin(object_detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    object_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()