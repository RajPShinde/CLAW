#!/usr/bin/env python3.8
import time
import board
import adafruit_bus_device.spi_device as spidev
import neopixel_spi as neopixel
import busio

NUM_PIXELS = 6
PIXEL_ORDER = neopixel.GRB
COLORS = (0xFF0000, 0x00FF00, 0x0000FF)
DELAY = 0.1

spi = busio.SPI(board.SCK_1, board.MOSI_1, board.MISO_1)

pixels = neopixel.NeoPixel_SPI(
    spi, NUM_PIXELS, pixel_order=PIXEL_ORDER, auto_write=False
)

while True:
    for color in COLORS:
        for i in range(NUM_PIXELS):
            pixels[i] = 0x00FF00
            pixels.show()
            time.sleep(DELAY)
            pixels.fill(0)
