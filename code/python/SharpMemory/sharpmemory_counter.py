"""
This demo will fill the screen with white, draw a black box on top
and then print Hello World! in the center of the display

This example is for use on (Linux) computers that are using CPython with
Adafruit Blinka to support CircuitPython libraries. CircuitPython does
not support PIL/pillow (python imaging library)!
"""

import board
import busio
import digitalio
#from PIL import Image, ImageDraw, ImageFont
import adafruit_sharpmemorydisplay
import time

# Colors
BLACK = 0
WHITE = 255

# Parameters to Change
BORDER = 3
FONTSIZE = 50

spi = busio.SPI(board.SCK, MOSI=board.MOSI)
scs = digitalio.DigitalInOut(board.D22)  # inverted chip select

# display = adafruit_sharpmemorydisplay.SharpMemoryDisplay(spi, scs, 96, 96)
display = adafruit_sharpmemorydisplay.SharpMemoryDisplay(spi, scs, 400, 240)

# Clear display.
display.fill(1)
display.show()

# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
#image = Image.new("1", (display.width, display.height))

# Get drawing object to draw on image.
#draw = ImageDraw.Draw(image)

# Draw a black background
#draw.rectangle((0, 0, display.width, display.height), outline=BLACK, fill=BLACK)

# Draw a smaller inner rectangle
#draw.rectangle(
#     (BORDER, BORDER, display.width - BORDER - 1, display.height - BORDER - 1),
#     outline=WHITE,
#     fill=WHITE,
# )

# Load a TTF font.
#font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 15)
#counterFont = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", FONTSIZE)

count=0

while(True):
  # Draw Some Text
  count=count+1
#   text = str(count)
#   (font_width, font_height) = counterFont.getsize(text)
#   draw.text(
#      (display.width // 2 - font_width // 2, display.height // 2 - font_height // 2),
#       text,
#       font=counterFont,
#       fill=BLACK,
#   )
  
  #draw.text(
  #    (15, 15),
  #    "DasBoot AP:",
  #    font=font,
  #    fill=BLACK,
  #)

  # Display image
 
  #display.image(image)
  #display.show()
  display.fill(1)
  display.text("DB: "+ str(count/10), 40, 75, 0, size=5)
  display.show()
  
  #print(count)
  #display.fill(1)
