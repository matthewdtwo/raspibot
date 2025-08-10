from flask import Flask, send_file, Response

from picamera2 import Picamera2
from libcamera import Transform
import io
import time

picam2 = Picamera2()
config = picam2.create_still_configuration(transform=Transform(vflip=1, hflip=1))
picam2.configure(config)

picam2.start()
time.sleep(1)

app = Flask(__name__)



@app.route('/')
def capture_image():
    # Capture an image
    data = io.BytesIO()
    picam2.capture_file(data, format='jpeg')
    data.seek(0)
    data.seek(0)
    return send_file(data, mimetype='image/jpeg')


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)