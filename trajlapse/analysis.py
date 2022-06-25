# Module which performs image analysis

from math import log2
import numpy
from PIL import Image
import numexpr
import exiv2
import logging
logger = logging.getLogger(__name__)

from .exposure import lens
numexpr.set_num_threads(1)


class Analyzer:
    __slots__ = ('shape', 'mask', 'linearize', 'Lmat', 'Lary')

    def __init__(self, resolution):
        self.shape = resolution
        self.mask = None
        self.init_mask()
        self.linearize = numexpr.NumExpr("where(x<=10,x/3294.6,((x/255.0+0.055)/1.055)**2.4)")
        self.Lmat = numpy.array([21.26, 71.52, 7.22], dtype=numpy.float32)
        self.Lary = numpy.empty(self.mask[0].size, dtype=numpy.float32)

    def init_mask(self):
#        c = (numpy.array(self.shape) / 2)
#        y, x = numpy.ogrid[-c[0]:self.shape[0] - c[0], -c[1]:self.shape[1] - c[1]]
#        r2 = x * x + y * y
        p = (numpy.arange(numpy.prod(self.shape)) % 29 == 0).reshape(self.shape)
        p[self.shape[0] // 3:2 * self.shape[0] // 3, self.shape[1] // 3:2 * self.shape[1] // 3] = 1
        self.mask = numpy.where(p)

    def process(self, filename):
        # try:
        #     results = self.read_exif(filename)
        # except Exception:
        #     results = {"exif": "corrupted"}
        results = {}
        ary = numpy.asarray(Image.open(filename))
        results["filename"] = filename
        results["delta_Ev"] = self.calc_expo(ary)
        results["delta_rb"] = self.calc_awb(ary)
        return results

    __call__ = process
    def calc_expo(self, ary):
        """
        :param ary: input array (h,w,3)
        :return: exposure correction in + or - Ev
        """
        lrgb = self.linearize(ary[self.mask])
        numpy.dot(lrgb.astype(numpy.float32), self.Lmat, out=self.Lary)
        self.Lary += 0.5
        hist = numpy.bincount(self.Lary.astype(numpy.int8))
        maxi = max(1, numpy.argmax(hist))  # median, mean or mod ?
        return -log2(maxi / 18)  # 18% for a perfect exposition

    def calc_awb(self, ary):
        """Calculate the red and blue gain correction for auto-white balance"""
        lary = ary.reshape(-1, 3)
        r = numpy.cumsum(numpy.bincount(lary[:, 0]))
        g = numpy.cumsum(numpy.bincount(lary[:, 1]))
        b = numpy.cumsum(numpy.bincount(lary[:, 2]))
        rg = numpy.argmin(abs(r - 0.995 * r[-1])) + 1
        gg = numpy.argmin(abs(g - 0.995 * g[-1])) + 1
        bg = numpy.argmin(abs(b - 0.995 * b[-1])) + 1
        return rg / gg, bg / gg

    def read_exif(self, filename):
        """Parse EXIF metadata to calculate exposure"""
        image = exiv2.ImageFactory.open(filename)
        image.readMetadata()
        exif = image.exifData()
        results = {
        "iso": exif["Exif.Photo.ISOSpeedRatings"].toLong(),
        "speed": exif["Exif.Photo.ShutterSpeedValue"].toFloat(),
        "Ev_exif": exif["Exif.Photo.BrightnessValue"].toFloat()
        }
        results["Ev_calc"] = lens.calc_EV(results["speed"], iso=results["iso"])
        return results
