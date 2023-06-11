#!/usr/bin/env python3.8
import time
import board
import adafruit_bus_device.spi_device as spidev
import neopixel_spi as neopixel
import busio
import threading

NUM_PIXELS = 6
PIXEL_ORDER = neopixel.GRB
statusColors = (0xFF0000, 0x00FF00, 0x0000FF)
legPhaseColors = (0xFF0000, 0x00FF00, 0x0000FF)

spi = busio.SPI(board.SCK_1, board.MOSI_1, board.MISO_1)

pixels = neopixel.NeoPixel_SPI(spi, NUM_PIXELS, pixel_order=PIXEL_ORDER, auto_write=False)

pixels[5] = 0xFFFFFF

def status():
    while True:
        pixels[4] = 0xFF0000
        pixels.show()
        time.sleep(0.1)
        pixels[4] = 0x000000
        pixels.show()
        time.sleep(0.1)

        pixels[4] = 0xFF0000
        pixels.show()
        time.sleep(0.1)
        pixels[4] = 0x000000
        pixels.show()
        time.sleep(1)

t1 = threading.Thread(target=status)
t1.start()

while True:

    pixels[0] = 0x000000
    pixels[1] = 0x000000
    pixels[2] = 0x000000
    pixels[3] = 0x000000
    pixels.show()
    time.sleep(0.8)

    pixels[0] = 0x00FF00
    pixels[1] = 0x00FF00
    pixels[2] = 0x00FF00
    pixels[3] = 0x00FF00
    pixels.show()
    time.sleep(0.10)



    pixels[0] = 0x000000
    pixels[1] = 0x000000
    pixels[2] = 0x000000
    pixels[3] = 0x000000
    pixels.show()
    time.sleep(0.30)



    pixels[0] = 0x00FF00
    pixels[1] = 0x00FF00
    pixels[2] = 0x00FF00
    pixels[3] = 0x00FF00
    pixels.show()
    time.sleep(0.10)



# t1.join()
