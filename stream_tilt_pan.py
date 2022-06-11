#!/usr/bin/env python3
# coding: utf-8
import time
import sys
from collections import OrderedDict
import logging
import io
from math import atan2, pi
from threading import Condition, Event
from argparse import ArgumentParser
import os
import json
import signal

from picamera import PiCamera
import bottle
import datetime

from trajlapse import servo
from trajlapse.positioner import Positioner, Position
from trajlapse.exposure import lens
from trajlapse.accelero import Accelerometer

print(lens)

bottle.debug(True)
root = os.path.dirname(os.path.abspath(__file__))


class Server(object):
    page = """
<html>
<header>
<META HTTP-EQUIV="refresh" CONTENT="1; url=/">
<title> pan={pan} tilt={tilt} Ev= {Ev}</title>
<style>
{style}
</style>
</header>
<body>
 <body>
    <div class="row">
        <div class="column" style="background-color:#FFB695;">
<center>
<p>
<a href="pan_min" title="pan -90"> |&lt </a>
<a href="pan_-10" title="pan -=10"> &lt&lt </a>
<a href="pan_-01" title="pan -=01"> &lt </a>
<a href="pan_0" title="pan =0"> pan center </a>
<a href="pan_+1" title="pan +=01"> &gt </a>
<a href="pan_+10" title="pan +=10"> &gt&gt </a>
<a href="pan_max" title="pan +180"> &gt| </a>
</p><p>
<a href="tilt_min" title="tilt = -90"> |&lt </a>
<a href="tilt_-10" title="tilt -= 10"> &lt&lt </a>
<a href="tilt_-1" title="tilt -= 01"> &lt </a>
<a href="tilt_0" title="tilt = 0"> Tilt center </a>
<a href="tilt_+1" title="tilt += 1"> &gt </a>
<a href="tilt_+10" title="tilt += 10"> &gt&gt </a>
<a href="tilt_max" title="tilt +90"> &gt| </a>
</p>
<p><img src="stream.jpg" width="800" height="600" title="{date_time}"/></p>
<p><a href="save" title="add position to trajectory">Save pos</a></p>
</center>
        </div>
        <div class="column" style="background-color:#96D1CD;">
<p> All metadata </p>
<ul>
<li>Camera: {revision}</li>
<li>Tilt: {tilt}°</li>
<li>Pan: {pan}°</li>
<li>EV: {Ev}</li>
<li>Focal: {focal}mm</li>
<li>Aperture: F/{aperture}</li>
<li>speed: {speed} 1/s</li>
<li>ISO: {iso} </li>
<li>awb_gains: {awb_gains}</li>
<li>analog_gain: {analog_gain}</li>
<li>digital_gain: {digital_gain}</li>
<li>exposure_compensation: {exposure_compensation}</li>
<li>exposure_speed: {exposure_speed}</li>
<li>framerate: {framerate}</li>
<li>shutter_speed: {shutter_speed}</li>
<li>date_time: {date_time}</li>
<li>Gravity: {gravity}</li>
<li>Measured tilt: {meas_tilt}°</li>
<li>Measured roll: {meas_roll}°</li>
</ul>
        </div>
    </div>

</body>
</html>
    """

    def __init__(self, img_dir="images", ip="0.0.0.0", port=80):
        self.img_dir = img_dir
        if not os.path.exists(self.img_dir):
            os.mkdir(self.img_dir)
        self.trajectory = []
        self.traj_file = datetime.datetime.now().strftime("%Y-%m-%d-%Hh%Mm%Ss.json")
        self.ip = ip
        self.port = port
        self.bottle = bottle.Bottle()
        self.setup_routes()
        self.quit_event = Event()
        signal.signal(signal.SIGINT, self.quit)
        self.acc = Accelerometer(quit_event=self.quit_event)
        self.acc.start()
        self.positioner = Positioner(servo.pan, servo.tilt, locks=[self.acc.lock, ])
        self.default_pos = Position(0, 0)
        self.current_pos = None
        self.camera = None
        self.streamout = None
        self.resolution = (800, 600)
        self.avg_wb = 200
        self.avg_ev = 200
        self.histo_ev = []
        self.wb_red = []
        self.wb_blue = []
        self.last_image = None
        self.setup_cam()

    def __del__(self):
        if self.camera:
            self.camera.stop_recording()
        self.camera= self.streamout = None

    def quit(self, *arg, **kwarg):
        self.quit_event.set()
        self.bottle.close()
        time.sleep(1)
        raise SystemExit("Ended by user")
        sys.exit(1)

    def setup_routes(self):
        self.bottle.route('/images/:filename', callback=self.server_static)
        self.bottle.route('/', callback=self.index)
        self.bottle.route('/pan_min', callback=self.pan_min)
        self.bottle.route('/pan_-10', callback=self.pan_sub10)
        self.bottle.route('/pan_-01', callback=self.pan_sub1)
        self.bottle.route('/pan_0' , callback=self.pan_center)
        self.bottle.route('/pan_+1', callback=self.pan_add1)
        self.bottle.route('/pan_+10', callback=self.pan_add10)
        self.bottle.route('/pan_max', callback=self.pan_max)
        self.bottle.route('/tilt_min', callback=self.tilt_min)
        self.bottle.route('/tilt_-10', callback=self.tilt_sub10)
        self.bottle.route('/tilt_-1', callback=self.tilt_sub1)
        self.bottle.route('/tilt_0', callback=self.tilt_center)
        self.bottle.route('/tilt_+1', callback=self.tilt_add1)
        self.bottle.route('/tilt_+10' , callback=self.tilt_add10)
        self.bottle.route('/tilt_max', callback=self.tilt_max)
        self.bottle.route('/reload', callback=self.move)
        self.bottle.route("/save", callback=self.save)
        self.bottle.route("/stream.jpg", callback=self.stream)

        headers = {'Age': 0,
                   'Cache-Control': 'no-cache, private',
                   'Pragma': 'no-cache',
                   'Content-Type': 'multipart/x-mixed-replace; boundary=FRAME'
                   }
#        for k, v in headers.items():
#            self.bottle.response.add_header(k, v)

    def server_static(self, filename):
        return bottle.static_file(filename, self.img_dir)

    def index(self):
        return self.move()

    def start(self):
        """Start the serveur (does not return):::"""
        self.bottle.run(host=self.ip, port=self.port)

    def move(self, new_pos=None):
        if new_pos is None:
            if self.current_pos is None:
                new_pos = self.default_pos
            else:
                new_pos = self.current_pos
        if new_pos.tilt < -90:
            new_pos = Position(new_pos.pan, -90)
        elif new_pos.tilt > 90:
            new_pos = Position(new_pos.pan, 90)
        if new_pos.pan < -90:
            new_pos = Position(-90, new_pos.tilt)
        elif new_pos.pan > 180:
            new_pos = Position(180, new_pos.tilt)
        if new_pos != self.current_pos:
            self.goto_pos(new_pos)

        dico = self.capture()
        dico["focal"] = lens.focal
        dico["aperture"] = lens.aperture
        dico["tilt"] = new_pos.tilt
        dico["pan"] = new_pos.pan
        if dico["Ev"] != "?":
            self.histo_ev.append(dico["Ev"])
        if dico["awb_gains"] != [0, 0]:
            red, blue = dico["awb_gains"]
            self.wb_red.append(red)
            self.wb_blue.append(blue)
        if len(self.wb_red) > self.avg_wb:
            self.wb_red = self.wb_red[-self.avg_wb:]
            self.wb_blue = self.wb_blue[-self.avg_wb:]
        if len(self.histo_ev) > self.avg_ev:
            self.histo_ev = self.histo_ev[-self.avg_ev:]
        grav = self.acc.get()
        dico["gravity"] = grav[:3]
        if grav:
            m_tilt = round(180.0 * atan2(-grav.y, -grav.z) / pi, 2)
            m_roll = round(180.0 * atan2(-grav.x, -grav.z) / pi, 2)
        else:
            m_roll = m_tilt = "?"
        dico["meas_tilt"] = m_tilt
        dico["meas_roll"] = m_roll
        webpage = self.page.format(**dico)
        self.current_pos = new_pos
        return webpage

    def goto_pos(self, pos):
        self.positioner.goto(pos)
        # self.positioner.stop_motors()

    def pan_min(self):
        pos = Position(-90, self.current_pos.tilt)
        return self.move(pos)

    def pan_max(self):
        pos = Position(+180, self.current_pos.tilt)
        return self.move(pos)

    def tilt_min(self):
        pos = Position(self.current_pos.pan, -90)
        return self.move(pos)

    def tilt_max(self):
        pos = Position(self.current_pos.pan, +90)
        return self.move(pos)

    def pan_sub1(self):
        pos = Position(self.current_pos.pan - 1 , self.current_pos.tilt)
        return self.move(pos)

    def pan_add1(self):
        pos = Position(self.current_pos.pan + 1, self.current_pos.tilt)
        return self.move(pos)

    def tilt_sub1(self):
        pos = Position(self.current_pos.pan, self.current_pos.tilt - 1)
        return self.move(pos)

    def tilt_add1(self):
        pos = Position(self.current_pos.pan, self.current_pos.tilt + 1)
        return self.move(pos)

    def pan_sub10(self):
        pos = Position(self.current_pos.pan - 10, self.current_pos.tilt)
        return self.move(pos)

    def pan_add10(self):
        pos = Position(self.current_pos.pan + 10, self.current_pos.tilt)
        return self.move(pos)

    def tilt_sub10(self):
        pos = Position(self.current_pos.pan, self.current_pos.tilt - 10)
        return self.move(pos)

    def tilt_add10(self):
        pos = Position(self.current_pos.pan, self.current_pos.tilt + 10)
        return self.move(pos)

    def pan_center(self):
        pos = Position(0, self.current_pos.tilt)
        return self.move(pos)

    def tilt_center(self):
        pos = Position(self.current_pos.pan, 0)
        return self.move(pos)

    def setup_cam(self):
        self.streamout = StreamingOutput()
        self.camera= PiCamera(resolution=self.resolution, framerate=30)  # , sensor_mode=3)
        self.camera.awb_mode = "auto"
        self.camera.exposure_mode= "nightpreview"
        self.camera.start_recording(self.streamout, format='mjpeg')

    def get_metadata(self):
        metadata = {"iso": float(self.camera.iso),
                    "analog_gain": float(self.camera.analog_gain),
                    "awb_gains": [float(i) for i in self.camera.awb_gains],
                    "digital_gain": float(self.camera.digital_gain),
                    "exposure_compensation": float(self.camera.exposure_compensation),
                    "exposure_speed": float(self.camera.exposure_speed),
                    "speed": 1e6/float(self.camera.exposure_speed),
                    "exposure_mode": self.camera.exposure_mode,
                    "framerate": float(self.camera.framerate),
                    "revision": self.camera.revision,
                    "shutter_speed": float(self.camera.shutter_speed),
                    "aperture": lens.aperture,
                    "focal_length": lens.focal,
                    "resolution": self.camera.resolution}
        if metadata['revision'] == "imx219":
            metadata['iso_calc'] = 54.347826086956516 * metadata["analog_gain"] * metadata["digital_gain"]
        if metadata['revision'] == "imx477":
            metadata['iso_calc'] = 40 * metadata["analog_gain"] * metadata["digital_gain"]
        else:
            metadata['iso_calc'] = 100.0 * metadata["analog_gain"] * metadata["digital_gain"]
        try:
            metadata['Ev'] = lens.calc_EV( 1e6 / metadata["shutter_speed"], metadata["digital_gain"]*metadata["analog_gain"])
        except:
            metadata["Ev"] = "?"
        return metadata


    def capture(self):
        dico = self.get_metadata()
        dico["date_time"] = datetime.datetime.now().strftime("%Y-%m-%d-%Hh%Mm%Ss")
        dico["style"] = """{box-sizing: border-box;}
.column {float: left; width: 50%;}
.row:after { content: ""; display: table; clear: both; }
}"""
        return dico

    def save(self):
        self.trajectory.append(self.current_pos)
        traj = [{"tilt": i.tilt, "pan": i.pan, "move": 60, "stay":10}
                for i in self.trajectory]
        camera = OrderedDict((("sensor_mode", 3),
                              ("warmup", 10),
                              ("framerate", 1),
                              ("avg_wb", self.avg_wb),
                              ("avg_ev", self.avg_ev),
                              ("histo_ev", self.histo_ev),
                              ("wb_red", self.wb_red),
                              ("wb_blue", self.wb_blue),))
        dico = OrderedDict((("trajectory", traj),
                            ("delay", 10),
                            ("folder", "."),
                            ("camera", camera),
                            ))

        with open(self.traj_file, "w") as f:
            f.write(json.dumps(dico, indent=2))
        return self.move()

    def stream(self):
        """write the stream"""
        try:
            while not self.quit_event.is_set():
                with self.streamout.condition:
                    self.streamout.condition.wait()
                    frame = self.streamout.frame
                    return frame
        except Exception as e:
            logging.warning('Removed streaming client: %s', e)


class StreamingOutput(object):
    """This class handles the stream"""

    def __init__(self):
        self.frame = None
        self.buffer = io.BytesIO()
        self.condition = Condition()

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # New frame, copy the existing buffer's content and notify all
            # clients it's available
            self.buffer.truncate()
            with self.condition:
                self.frame = self.buffer.getvalue()
                self.condition.notify_all()
            self.buffer.seek(0)
        return self.buffer.write(buf)


def main():
    parser = ArgumentParser("serveur", description="open a web server on a given directory")
    parser.add_argument("-p", "--port", help="open specified port", type=int, default=8080)
    parser.add_argument("-i", "--ip", help="Listen on this port", type=str, default="0.0.0.0")
    parser.add_argument("-d", "--directory", help="Directory containing images", type=str, default="images")
    args = parser.parse_args()
    server = Server(img_dir=args.directory, ip=args.ip, port=args.port)
    server.start()


if __name__ == '__main__':
    main()
