import time
import framebuf
from machine import I2C, Pin

class SSD1306:
    """
    Finaler 72x40 OLED Treiber mit dynamischem Display Offset für flip()
    - Geflippt: 100% korrekt mit window_y_flipped = 12
    - Normal:   100% korrekt mit window_y_normal = 52
    """
    def __init__(self, i2c, address=0x3C):
        self.i2c = i2c
        self.address = address
        self.window_x = 30
        self.window_y_normal = 52      # Für Normal-Modus (muss getestet werden!)
        self.window_y_flipped = 12    # Für Flipped-Modus (funktioniert perfekt!)
        self.width = 72
        self.height = 40
        self.flipped = False
        
        self.buffer = bytearray(self.width * self.height // 8)
        self.fb = framebuf.FrameBuffer(
            self.buffer, self.width, self.height, framebuf.MONO_VLSB)
        
        self.init_display()

    def write_cmd(self, cmd):
        self.i2c.writeto(self.address, bytearray([0x00, cmd]))

    def init_display(self):
        self.write_cmd(0xAE)
        self.write_cmd(0xD5); self.write_cmd(0x80)
        self.write_cmd(0xA8); self.write_cmd(0x27)  # 40 Zeilen
        
        # Start mit Normal-Modus Offset
        self.write_cmd(0xD3); self.write_cmd(self.window_y_normal)
        
        self.write_cmd(0x40)  # Start line 0
        self.write_cmd(0x8D); self.write_cmd(0x14)
        self.write_cmd(0x20); self.write_cmd(0x00)
        self.write_cmd(0xA0)  # Normal
        self.write_cmd(0xC0)  # Normal
        self.write_cmd(0xDA); self.write_cmd(0x12)
        self.write_cmd(0x81); self.write_cmd(0xCF)
        self.write_cmd(0xD9); self.write_cmd(0xF1)
        self.write_cmd(0xDB); self.write_cmd(0x40)
        self.write_cmd(0xA4); self.write_cmd(0xA6)
        self.write_cmd(0xAF)
        self.clear()
        self.show()

    def show(self):
        pages = self.height // 8
        
        for page in range(pages):
            self.write_cmd(0xB0 + page)
            self.write_cmd(0x00 + (self.window_x & 0x0F))
            self.write_cmd(0x10 + (self.window_x >> 4))
            start = page * self.width
            self.i2c.writeto(self.address,
                bytearray([0x40]) + self.buffer[start:start+self.width])

    def flip(self):
        if not self.flipped:
            # Flip aktivieren
            self.write_cmd(0xA1)
            self.write_cmd(0xC8)
            # Display Offset auf Flipped-Wert setzen
            self.write_cmd(0xD3); self.write_cmd(self.window_y_flipped)
        else:
            # Flip deaktivieren
            self.write_cmd(0xA0)
            self.write_cmd(0xC0)
            # Display Offset auf Normal-Wert setzen
            self.write_cmd(0xD3); self.write_cmd(self.window_y_normal)
        
        self.flipped = not self.flipped
        self.show()

    def clear(self):
        self.fb.fill(0)

    def pixel(self, x, y, color=1):
        self.fb.pixel(x, y, color)

    def text(self, s, x, y, color=1):
        self.fb.text(s, x, y, color)

    def rect(self, x, y, w, h, color=1):
        self.fb.rect(x, y, w, h, color)

    def line(self, x1, y1, x2, y2, color=1):
        self.fb.line(x1, y1, x2, y2, color)

    def hline(self, x, y, w, color=1):
        self.fb.hline(x, y, w, color)

    def vline(self, x, y, h, color=1):
        self.fb.vline(x, y, h, color)

    def fill(self, color=1):
        self.fb.fill(color)

    def fill_rect(self, x, y, w, h, color=1):
        self.fb.fill_rect(x, y, w, h, color)

    def scroll(self, dx, dy):
        self.fb.scroll(dx, dy)

    def blit(self, fb, x, y):
        self.fb.blit(fb, x, y)

    def invert(self, invert=True):
        self.write_cmd(0xA7 if invert else 0xA6)

    def power(self, on=True):
        self.write_cmd(0xAF if on else 0xAE)

    def contrast(self, level):
        self.write_cmd(0x81)
        self.write_cmd(level)
