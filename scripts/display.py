#!/usr/bin/env python3
from luma.core.interface.serial import i2c, spi, pcf8574
from luma.core.interface.parallel import bitbang_6800
from luma.core.render import canvas
from luma.oled.device import sh1106
from time import sleep
import sys
from pathlib import Path
import av
import PIL

serial = i2c(port=1, address=0x3C)

device = sh1106(serial)

video_path = str(Path(__file__).resolve().parent.joinpath('logo_screen_white.mp4'))

clip = av.open(video_path)

for frame in clip.decode(video=0):

    img = frame.to_image()
    if img.width != device.width or img.height != device.height:

        size = device.width, device.height
        img = img.resize(size, PIL.Image.ANTIALIAS)

    device.display(img.convert(device.mode))
