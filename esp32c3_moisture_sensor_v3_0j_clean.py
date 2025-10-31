"""
ESP32-C3-OLED Kapazitiver Feuchtesensor - Version 3.0j 31-10-2025
Button-Poll Version mit geschachteltem MQTT JSON

ÄNDERUNGEN in v3.0j:
- ERROR_UNDERLOAD und ERROR_OVERLOAD Zonen hinzugefügt
- Out-of-Range Handling für ADC-Werte (ADC < 420 → "UL", ADC > 850 → "OL")
- determine_moisture_status() robuster gegen None-Rückgaben
- Fallback "ERR" für unerwartete Werte

HARDWARE-KONFIGURATION:
PIN-BELEGUNG:
GPIO0: Sensor Power Control (OUTPUT)
GPIO1: ADC1_CH1 - U-SENSOR Messung (INPUT, Analog)
GPIO2: ADC1_CH2 - VBAT Messung (INPUT, Analog)
GPIO3: Button mit Pull-Up (INPUT, Digital)
GPIO5: I2C SDA (OLED Display)
GPIO6: I2C SCL (OLED Display)
GPIO8: Interne LED (OUTPUT, LOW active)

FEUCHTIGKEITSZONEN (NEU in v3.0j):
ADC > 850: "OL" - Overload (Sensor defekt, nicht verbunden)
ADC 767-850: "sDRY" - Sehr trocken (Gießkanne nötig)
ADC 684-766: "DRY" - Trocken
ADC 587-683: "OK" - Optimal
ADC 504-586: "WET" - Feucht
ADC 420-503: "sWET" - Sehr feucht
ADC < 420: "UL" - Underload (Sensor im Wasser/defekt)

MQTT JSON STRUKTUR (mit Untergruppen):
{
  "device": { "devicename", "timestamp", "measurement_count", "software_version" },
  "battery": { "b_voltage", "b_adc", "b_percent" },
  "sensor": { "s_voltage", "s_adc", "s_percent" },
  "s_moisture_status": "OL" | "sDRY" | "DRY" | "OK" | "WET" | "sWET" | "UL" | "ERR"
}

Autor: Mit Out-of-Range Handling (OL/UL)
Datum: Oktober 2025
Version: 3.0j
"""

import machine
import time
from machine import Pin, ADC, I2C, deepsleep, reset_cause, DEEPSLEEP_RESET, RTC
import network
import ujson
import gc

# ============================================================================
# KONFIGURATION
# ============================================================================

VERSION = "3.0j"

# WLAN
WIFI_SSID = "FritzBoxFonWLAN"
WIFI_PASSWORD = "3373695987550335"
WIFI_TIMEOUT = 15
TX_POWER = 5
RETRY_COUNT = 5

# MQTT
MQTT_BROKER = "192.168.178.35"
MQTT_PORT = 1883
MQTT_USER = ""
MQTT_PASSWORD = ""
MQTT_CLIENT_ID = "ESP32-C3-OLED_Feuchtesensor_01"
MQTT_TOPIC = "home/feuchtesensor/01"

# Display
DISPLAY_TIMEOUT = 5
OLED_WIDTH = 72
OLED_HEIGHT = 40

# Button-Poll vor Sleep
BUTTON_POLL_TIME = 5

# Deep Sleep
SLEEP_TIME = 60

# Sensor Kalibrierung
MOISTURE_DRY = 850  # 0% Feuchtigkeit
MOISTURE_WET = 420  # 100% Feuchtigkeit

# Battery Kalibrierung
BATTERY_MAX = 4.2   # 100%
BATTERY_MIN = 3.0   # 0%

# Spannungsteiler-Faktoren
SENSOR_VOLTAGE_FACTOR = 2.47
BATTERY_VOLTAGE_FACTOR = 3.3

# ============================================================================
# GLOBALE VARIABLEN
# ============================================================================

measurement_count = 0
display = None
wlan = None
mqtt_client = None
i2c = None
adc_sensor = None
adc_battery = None
sensor_power = None
led_internal = None
wake_button = None

# ====================================================================
# SENSOR-KALIBRIERUNG UND ICON-ZUORDNUNG - Version 3.0j
# Mit Out-of-Range Handling für OL/UL
# ====================================================================

MOISTURE_ZONES = [
    # (Name,              ADC_Min, ADC_Max, Icon_ID, Status_Text)
    ("ERROR_OVERLOAD",   851,     9999,    6,       "OL"),         # Überload - über Messbereich
    ("SEHR_TROCKEN",     767,     850,     5,       "sDRY"),       # V: Gießkanne
    ("TROCKEN",          684,     766,     4,       "DRY"),        # IV: Blume 1R
    ("OPTIMAL",          587,     683,     3,       "OK"),         # III: Blume 2B
    ("FEUCHT",           504,     586,     2,       "WET"),        # II: Blume 1L
    ("SEHR_FEUCHT",      420,     503,     1,       "sWET"),       # I: Wellen
    ("ERROR_UNDERLOAD",  0,       419,     6,       "UL"),         # Underload - unter Messbereich
]

# Legacy-Konstanten für Kompatibilität
MOISTURE_OK_MIN = 587
MOISTURE_OK_MAX = 683
ZONE_STEP_SIZE = 84

# ====================================================================
# 5-STUFEN DISPLAY-SYMBOLE (40x40 Pixel)
# ====================================================================

# SYMBOL 1: WELLENLINIEN - SEHR_FEUCHT (ADC 420-503)
ICON_WAVES = bytearray([
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xC0, 0x00, 0x03, 0xC0, 0x04, 0x20, 0x00, 0x04, 0x20, 0x08, 0x10, 0x00, 0x08, 0x10,
0x10, 0x08, 0x00, 0x10, 0x08, 0x20, 0x04, 0x00, 0x20, 0x04, 0x20, 0x04, 0x00, 0x20, 0x04, 0x20, 0x06, 0x00, 0x60, 0x04,
0x03, 0xC2, 0x00, 0x43, 0xC0, 0x04, 0x22, 0x00, 0x44, 0x20, 0x08, 0x11, 0x00, 0x88, 0x10, 0x10, 0x08, 0x81, 0x10, 0x08,
0x20, 0x04, 0x42, 0x20, 0x04, 0x20, 0x04, 0x3C, 0x20, 0x04, 0x20, 0x06, 0x00, 0x60, 0x04, 0x03, 0xC2, 0x00, 0x43, 0xC0,
0x04, 0x22, 0x00, 0x44, 0x20, 0x08, 0x11, 0x00, 0x88, 0x10, 0x10, 0x08, 0x81, 0x10, 0x08, 0x20, 0x04, 0x42, 0x20, 0x04,
0x20, 0x04, 0x3C, 0x20, 0x04, 0x20, 0x06, 0x00, 0x60, 0x04, 0x00, 0x02, 0x00, 0x40, 0x00, 0x00, 0x02, 0x00, 0x40, 0x00,
0x00, 0x01, 0x00, 0x80, 0x00, 0x00, 0x00, 0x81, 0x00, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
])

# SYMBOL 2: BLUME + 1 BLATT LINKS - FEUCHT (ADC 504-586)
ICON_FLOWER_1L = bytearray([
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x44, 0x00, 0x00, 0x00, 0x07, 0x83, 0xC0, 0x00,
0x00, 0x08, 0x82, 0x20, 0x00, 0x00, 0x10, 0xC6, 0x10, 0x00, 0x00, 0x10, 0x44, 0x10, 0x00, 0x00, 0x10, 0x7C, 0x10, 0x00,
0x00, 0x1C, 0xBA, 0x70, 0x00, 0x00, 0x27, 0x01, 0xC8, 0x00, 0x00, 0x41, 0xAB, 0x04, 0x00, 0x00, 0x41, 0x83, 0x04, 0x00,
0x00, 0x41, 0xAB, 0x04, 0x00, 0x00, 0x27, 0x01, 0xC8, 0x00, 0x00, 0x1C, 0xBA, 0x70, 0x00, 0x00, 0x10, 0x7C, 0x10, 0x00,
0x00, 0x10, 0x44, 0x10, 0x00, 0x00, 0x10, 0xC6, 0x10, 0x00, 0x00, 0x08, 0x82, 0x20, 0x00, 0x00, 0x07, 0x83, 0xC0, 0x00,
0x00, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00,
0x00, 0x1E, 0x38, 0x00, 0x00, 0x00, 0x61, 0xB8, 0x00, 0x00, 0x00, 0x80, 0x78, 0x00, 0x00, 0x00, 0x71, 0xB8, 0x00, 0x00,
0x00, 0x0E, 0x38, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00,
0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x00, 0x00,
0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
])

# SYMBOL 3: BLUME + 2 BLÄTTER - OPTIMAL (ADC 587-683)
ICON_FLOWER_2B = bytearray([
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x44, 0x00, 0x00, 0x00, 0x07, 0x83, 0xC0, 0x00,
0x00, 0x08, 0x82, 0x20, 0x00, 0x00, 0x10, 0xC6, 0x10, 0x00, 0x00, 0x10, 0x44, 0x10, 0x00, 0x00, 0x10, 0x7C, 0x10, 0x00,
0x00, 0x1C, 0xBA, 0x70, 0x00, 0x00, 0x27, 0x01, 0xC8, 0x00, 0x00, 0x41, 0xAB, 0x04, 0x00, 0x00, 0x41, 0x83, 0x04, 0x00,
0x00, 0x41, 0xAB, 0x04, 0x00, 0x00, 0x27, 0x01, 0xC8, 0x00, 0x00, 0x1C, 0xBA, 0x70, 0x00, 0x00, 0x10, 0x7C, 0x10, 0x00,
0x00, 0x10, 0x44, 0x10, 0x00, 0x00, 0x10, 0xC6, 0x10, 0x00, 0x00, 0x08, 0x82, 0x20, 0x00, 0x00, 0x07, 0x83, 0xC0, 0x00,
0x00, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00,
0x00, 0x1E, 0x38, 0x00, 0x00, 0x00, 0x61, 0xB8, 0x00, 0x00, 0x00, 0x80, 0x78, 0xF0, 0x00, 0x00, 0x71, 0xBB, 0x0C, 0x00,
0x00, 0x0E, 0x3C, 0x02, 0x00, 0x00, 0x00, 0x3B, 0x1C, 0x00, 0x00, 0x00, 0x38, 0xE0, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00,
0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x00, 0x00,
0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
])

# SYMBOL 4: BLUME + 1 BLATT RECHTS - TROCKEN (ADC 684-766)
ICON_FLOWER_1R = bytearray([
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x44, 0x00, 0x00, 0x00, 0x07, 0x83, 0xC0, 0x00,
0x00, 0x08, 0x82, 0x20, 0x00, 0x00, 0x10, 0xC6, 0x10, 0x00, 0x00, 0x10, 0x44, 0x10, 0x00, 0x00, 0x10, 0x7C, 0x10, 0x00,
0x00, 0x1C, 0xBA, 0x70, 0x00, 0x00, 0x27, 0x01, 0xC8, 0x00, 0x00, 0x41, 0xAB, 0x04, 0x00, 0x00, 0x41, 0x83, 0x04, 0x00,
0x00, 0x41, 0xAB, 0x04, 0x00, 0x00, 0x27, 0x01, 0xC8, 0x00, 0x00, 0x1C, 0xBA, 0x70, 0x00, 0x00, 0x10, 0x7C, 0x10, 0x00,
0x00, 0x10, 0x44, 0x10, 0x00, 0x00, 0x10, 0xC6, 0x10, 0x00, 0x00, 0x08, 0x82, 0x20, 0x00, 0x00, 0x07, 0x83, 0xC0, 0x00,
0x00, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00,
0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x38, 0xF0, 0x00, 0x00, 0x00, 0x3B, 0x0C, 0x00,
0x00, 0x00, 0x3C, 0x02, 0x00, 0x00, 0x00, 0x3B, 0x1C, 0x00, 0x00, 0x00, 0x38, 0xE0, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00,
0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x00, 0x00,
0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
])

# SYMBOL 5: GIESSKANNE - SEHR_TROCKEN (ADC 767-850)
ICON_WATERING = bytearray([
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xC0, 0x00,
0x00, 0x7F, 0xFF, 0xC0, 0x00, 0x00, 0x60, 0x00, 0xC0, 0x00, 0x00, 0x60, 0x00, 0xC0, 0x00, 0x00, 0x60, 0x00, 0xC0, 0x00,
0x00, 0x60, 0x00, 0xC0, 0x00, 0x00, 0x60, 0x00, 0xC0, 0x00, 0x1F, 0x60, 0x00, 0xC0, 0x00, 0x3F, 0xE0, 0x00, 0xC0, 0x00,
0x3B, 0xE0, 0x00, 0xC0, 0x06, 0x31, 0xE0, 0x00, 0xC0, 0x0E, 0x71, 0xE0, 0x00, 0xC0, 0x1E, 0x60, 0xE0, 0x00, 0xC0, 0x3C,
0x60, 0xE0, 0x00, 0xC0, 0x78, 0x60, 0xE0, 0x00, 0xC0, 0xF0, 0x60, 0xE0, 0x00, 0xC1, 0xE0, 0x60, 0xE0, 0x00, 0xC3, 0xC0,
0x60, 0xE0, 0x00, 0xC7, 0x80, 0x71, 0xE0, 0x00, 0xCF, 0x00, 0x31, 0xE0, 0x00, 0xDE, 0x00, 0x3B, 0xE0, 0x00, 0xFC, 0x00,
0x3F, 0xE0, 0x00, 0xF8, 0x00, 0x1F, 0x60, 0x00, 0xF0, 0x00, 0x00, 0x60, 0x00, 0xE0, 0x00, 0x00, 0x60, 0x00, 0xC0, 0x00,
0x00, 0x60, 0x00, 0xC0, 0x00, 0x00, 0x60, 0x00, 0xC0, 0x00, 0x00, 0x60, 0x00, 0xC0, 0x00, 0x00, 0x60, 0x00, 0xC0, 0x00,
0x00, 0x60, 0x00, 0xC0, 0x00, 0x00, 0x60, 0x00, 0xC0, 0x00, 0x00, 0x60, 0x00, 0xC0, 0x00, 0x00, 0x7F, 0xFF, 0xC0, 0x00,
0x00, 0x7F, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
])

# SYMBOL 6: GIESSKANNE - SEHR_TROCKEN (ADC 767-850)
ICON_WARNING = bytearray([
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,

])

def get_moisture_zone(adc_value):
    """Ermittelt die Feuchtigkeitszone basierend auf ADC-Wert"""
    for name, adc_min, adc_max, icon_id, status_text in MOISTURE_ZONES:
        if adc_min <= adc_value <= adc_max:
            return (name, icon_id, status_text)
    # Fallback - sollte nicht vorkommen
    if adc_value > 850:
        return ("SEHR_TROCKEN", 5, "sDRY")
    else:
        return ("SEHR_FEUCHT", 1, "sWET")

def get_icon_for_adc(adc_value):
    """Gibt Icon und Text für ADC-Wert zurück"""
    zone_name, icon_id, status_text = get_moisture_zone(adc_value)
    icons = {
        1: ICON_WAVES,
        2: ICON_FLOWER_1L,
        3: ICON_FLOWER_2B,
        4: ICON_FLOWER_1R,
        5: ICON_WATERING,
        6: ICON_WARNING,
    }
    return (icons[icon_id], status_text)

def print_moisture_zones():
    """Debug: Zeigt alle Feuchtigkeitszonen"""
    print("\n" + "="*70)
    print("FEUCHTIGKEITSZONEN UND ICON-ZUORDNUNG")
    print("="*70)
    print(f"{'Zone':<20} {'ADC-Bereich':<15} {'Icon':<10} {'Text':<12}")
    print("-"*70)
    for name, adc_min, adc_max, icon_id, text in MOISTURE_ZONES:
        adc_range = f"{adc_min}-{adc_max}"
        icon_name = f"Icon #{icon_id}"
        print(f"{name:<20} {adc_range:<15} {icon_name:<10} {text:<12}")
    print("="*70)

def draw_large_icon(display, icon, x, y):
    """Zeichnet 40×40 Pixel Icon"""
    for row in range(40):
        for byte_col in range(5):
            byte = icon[row * 5 + byte_col]
            for bit in range(8):
                if byte & (0x80 >> bit):
                    display.pixel(x + byte_col * 8 + bit, y + row, 1)

def show_icon_screen(display, adc_value):
    """Zeigt Icon-Screen basierend auf ADC-Wert (verwendet MOISTURE_ZONES Konfiguration)"""
    display.fill(0)
    icon, text = get_icon_for_adc(adc_value)
    draw_large_icon(display, icon, 16, 0)
    display.text(text, 38, 56)
    display.show()

# RTC für persistenten Counter
try:
    rtc = RTC()
    print("✓ RTC für Counter verfügbar")
except Exception as e:
    print("⚠ RTC nicht verfügbar: %s" % str(e))
    rtc = None

# ============================================================================
# RTC COUNTER FUNKTIONEN
# ============================================================================

def load_counter():
    """Lädt Counter aus RTC Memory"""
    if rtc is None:
        return 0
    try:
        data = rtc.memory()
        if data and len(data) >= 4:
            count = int.from_bytes(data[0:4], 'little')
            print("✓ Counter aus RTC geladen: %d" % count)
            return count
    except Exception as e:
        print("⚠ RTC Laden fehlgeschlagen: %s" % str(e))
    return 0

def save_counter(count):
    """Speichert Counter in RTC Memory"""
    if rtc is None:
        return False
    try:
        data = count.to_bytes(4, 'little')
        rtc.memory(data)
        print("✓ Counter in RTC gespeichert: %d" % count)
        return True
    except Exception as e:
        print("✗ RTC Speichern fehlgeschlagen: %s" % str(e))
        return False

# ============================================================================
# PROZENT-BERECHNUNGEN
# ============================================================================

def calc_sensor_percent(adc_value):
    """Berechnet Sensor-Feuchtigkeit in Prozent"""
    sensor_range = MOISTURE_DRY - MOISTURE_WET
    percent = ((MOISTURE_DRY - adc_value) / sensor_range) * 100
    return max(0, min(100, int(percent)))

def calc_battery_percent(voltage):
    """Berechnet Battery-Ladung in Prozent"""
    bat_range = BATTERY_MAX - BATTERY_MIN
    percent = ((voltage - BATTERY_MIN) / bat_range) * 100
    return max(0, min(100, int(percent)))

# ============================================================================
# HARDWARE-INITIALISIERUNG
# ============================================================================

def init_hardware():
    """Initialisiert Hardware-Komponenten"""
    global i2c, adc_sensor, adc_battery, sensor_power, led_internal, wake_button

    print("\n" + "="*70)
    print("HARDWARE-INITIALISIERUNG")
    print("="*70)

    # I2C
    try:
        i2c = I2C(0, sda=Pin(5), scl=Pin(6), freq=400000)
        devices = i2c.scan()
        if devices:
            print("✓ I2C initialisiert - Geräte: %s" % str([hex(d) for d in devices]))
        else:
            print("⚠ I2C initialisiert - Geräte nicht gefunden")
    except Exception as e:
        print("✗ I2C Fehler: %s" % str(e))
        i2c = None

    # ADC Sensor (GPIO1)
    try:
        adc_sensor = ADC(Pin(1))
        adc_sensor.atten(2)
        print("✓ ADC Sensor (GPIO1/ADC1_CH1, 6dB/0-2.0V)")
    except Exception as e:
        print("✗ ADC Sensor Fehler: %s" % str(e))
        adc_sensor = None

    # ADC Battery (GPIO2)
    try:
        adc_battery = ADC(Pin(2))
        adc_battery.atten(2)
        print("✓ ADC Battery (GPIO2/ADC1_CH2, 6dB/0-2.0V)")
    except Exception as e:
        print("✗ ADC Battery Fehler: %s" % str(e))
        adc_battery = None

    # Button (GPIO3)
    try:
        wake_button = Pin(3, Pin.IN, Pin.PULL_UP)
        print("✓ Button (GPIO3, PULL_UP)")
    except Exception as e:
        print("✗ Button Fehler: %s" % str(e))
        wake_button = DummyPin()

    # Sensor Power (GPIO0)
    try:
        sensor_power = Pin(0, Pin.OUT)
        sensor_power.off()
        print("✓ Sensor Power (GPIO0) - AUS")
    except Exception as e:
        print("✗ Sensor Power Fehler: %s" % str(e))
        sensor_power = DummyPin()

    # LED (GPIO8, LOW active)
    try:
        led_internal = Pin(8, Pin.OUT)
        led_internal.on()
        print("✓ LED (GPIO8, LOW active)")
    except Exception as e:
        print("✗ LED Fehler: %s" % str(e))
        led_internal = DummyPin()

    print("="*70)

class DummyPin:
    def on(self): pass
    def off(self): pass
    def value(self, *args): return 1 if not args else None

# ============================================================================
# DISPLAY FUNKTIONEN
# ============================================================================

def init_display():
    global display
    if i2c is None:
        print("⚠ Kein I2C - Display übersprungen")
        return False
    try:
        from ssd1306_7240_drv import SSD1306
        display = SSD1306(i2c)
        try:
            display.poweron()
            print("✓ Display poweron()")
        except AttributeError:
            try:
                display.power(True)
                print("✓ Display power(True)")
            except:
                print("✓ Display init")
        display.fill(0)
        display.text("ESP32-C3", 0, 0)
        display.text("OLED", 0, 10)
        display.text("Sensor V:", 0, 20)
        display.text(" " + VERSION, 0, 30)
        display.show()
        print("✓ Display initialisiert")
        time.sleep(1)
        return True
    except ImportError:
        print("⚠ ssd1306_7240_drv.py nicht gefunden")
        return False
    except Exception as e:
        print("✗ Display Fehler: %s" % str(e))
        return False

def display_values(vbat, vsensor, status, count):
    if display is None:
        return
    try:
        display.fill(0)
        display.text("Ba:%.4fV" % vbat, 0, 0)
        display.text("Se:%.4fV" % vsensor, 0, 10)
        display.text("%s" % status, 0, 20)
        if OLED_HEIGHT >= 40:
            display.text("#%d" % count, 0, 30)
        display.show()
    except Exception as e:
        print("✗ Display: %s" % str(e))

# ============================================================================
# LED FUNKTIONEN
# ============================================================================

def led_blink(times=1, delay=0.1):
    try:
        for _ in range(times):
            led_internal.off()
            time.sleep(delay)
            led_internal.on()
            time.sleep(delay)
    except:
        pass

def led_on():
    try:
        led_internal.off()
    except:
        pass

def led_off():
    try:
        led_internal.on()
    except:
        pass

# ============================================================================
# ADC MESSFUNKTIONEN
# ============================================================================

def read_battery_voltage():
    if adc_battery is None:
        return 0, 3.7
    try:
        led_on()
        readings = []
        for _ in range(10):
            readings.append(adc_battery.read())
            time.sleep_ms(10)
        led_off()
        avg_reading_b = sum(readings) // len(readings)
        V_REF = 2.0
        adc_voltage = avg_reading_b * (V_REF / 4095.0)
        battery_voltage = adc_voltage * BATTERY_VOLTAGE_FACTOR
        print(" Batterie: ADC=%d, V_ADC=%.4fV, VBAT=%.4fV" %
              (avg_reading_b, adc_voltage, battery_voltage))
        return avg_reading_b, battery_voltage
    except Exception as e:
        print("✗ Batterie: %s" % str(e))
        led_off()
        return 0, 3.7

def read_sensor_voltage():
    if adc_sensor is None:
        return 2000, 1.5
    try:
        sensor_power.on()
        print(" Sensor Power: EIN")
        time.sleep_ms(100)
        led_blink(2, 0.05)
        readings = []
        for _ in range(10):
            readings.append(adc_sensor.read())
            time.sleep_ms(10)
        sensor_power.off()
        print(" Sensor Power: AUS")
        avg_reading_s = sum(readings) // len(readings)
        V_REF = 2.0
        adc_voltage = avg_reading_s * (V_REF / 4095.0)
        sensor_voltage = adc_voltage * SENSOR_VOLTAGE_FACTOR
        print(" Sensor: ADC=%d, V_ADC=%.4fV, U_SENS=%.4fV" %
              (avg_reading_s, adc_voltage, sensor_voltage))
        return avg_reading_s, sensor_voltage
    except Exception as e:
        print("✗ Sensor: %s" % str(e))
        sensor_power.off()
        return 2000, 1.5

# ============================================================================
# STATUS FUNKTIONEN
# ============================================================================

def determine_moisture_status(avg_reading_s):
    """
    Bestimmt Feuchtigkeitsstatus basierend auf ADC-Wert
    Behandelt auch Out-of-Range Werte (OL/UL)
    """
    # Fallback: Wenn avg_reading_s None ist
    if avg_reading_s is None:
        return "ERR"
    
    # Schleife durch Zonen - erste Übereinstimmung gewinnt
    for name, adc_min, adc_max, icon_id, status_text in MOISTURE_ZONES:
        if adc_min <= avg_reading_s <= adc_max:
            return status_text
    
    # Sollte NICHT vorkommen (aber als Extra-Sicherung)
    print("✗ Unerwarteter ADC-Wert: %d" % avg_reading_s)
    return "ERR"

# ============================================================================
# BUTTON FUNCTIONS
# ============================================================================

def check_button_before_sleep(timeout_seconds=BUTTON_POLL_TIME):
    print("\nButton-Poll (%ds)..." % timeout_seconds)
    print("Taster: Messung sofort")
    print("Timeout: Deep Sleep" if SLEEP_TIME > 0 else "Timeout: Nächste Messung")
    countdown = timeout_seconds
    while countdown > 0:
        try:
            if wake_button.value() == 0:
                print("✓ Button gedrückt!")
                led_blink(2, 0.1)
                return True
        except:
            pass
        if countdown % 1 == 0:
            print(" %ds..." % countdown, end="")
            print("")
        time.sleep(0.1)
        countdown -= 0.1
    print("Timeout")
    return False

# ============================================================================
# WLAN FUNKTIONEN
# ============================================================================

def connect_wifi():
    global wlan
    print("\n" + "="*70)
    print("WLAN-VERBINDUNG")
    print("="*70)
    try:
        led_blink(3, 0.1)
        wlan = network.WLAN(network.STA_IF)
        print("Reset WiFi Interface...")
        wlan.active(False)
        time.sleep(0.5)
        wlan.active(True)
        time.sleep(0.5)
        wlan.config(pm=wlan.PM_NONE)
        wlan.config(txpower=TX_POWER)
        wlan.config(reconnects=RETRY_COUNT)
        if wlan.isconnected():
            wlan.disconnect()
            time.sleep(2)
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        start_time = time.time()
        while not wlan.isconnected() and (time.time() - start_time) < WIFI_TIMEOUT:
            time.sleep(0.5)
        if wlan.isconnected():
            config = wlan.ifconfig()
            print("✓ WLAN VERBUNDEN - IP: %s" % config[0])
            return True
        else:
            print("✗ WLAN Timeout")
            return False
    except Exception as e:
        print("✗ WLAN: %s" % str(e))
        return False

# ============================================================================
# MQTT FUNKTIONEN
# ============================================================================

def publish_mqtt(data):
    global mqtt_client
    try:
        from umqtt.simple import MQTTClient
        mqtt_client = MQTTClient(
            MQTT_CLIENT_ID,
            MQTT_BROKER,
            port=MQTT_PORT
        )
        mqtt_client.connect()
        json_data = ujson.dumps(data)
        mqtt_client.publish(MQTT_TOPIC, json_data)
        mqtt_client.disconnect()
        print("✓ MQTT publiziert")
        print(" %s" % json_data)
        return True
    except Exception as e:
        print("✗ MQTT: %s" % str(e))
        return False

# ============================================================================
# DEEP SLEEP FUNKTIONEN
# ============================================================================

def enter_deep_sleep():
    if display:
        try:
            display.fill(0)
            display.show()
            display.poweroff()
        except:
            pass
    led_off()
    try:
        sensor_power.off()
    except:
        pass
    if wlan:
        try:
            wlan.active(False)
        except:
            pass
    gc.collect()
    deepsleep(SLEEP_TIME * 1000)

# ============================================================================
# HAUPTPROGRAMM (mit geschachteltem JSON)
# ============================================================================

def main():
    global measurement_count

    print("\n" + "="*70)
    print(" ESP32-C3 FEUCHTESENSOR - VERSION " + VERSION)
    print(" Mit geschachteltem MQTT JSON (Untergruppen)")
    print("="*70)

    # Counter laden
    if reset_cause() == DEEPSLEEP_RESET:
        measurement_count = load_counter()
        print("✓ Wake from Sleep - Counter: %d" % measurement_count)
    else:
        print("✓ Power-On - Counter: 0")
        measurement_count = 0

    led_blink(3, 0.1)

    while True:
        measurement_count += 1
        print("\n" + "="*70)
        print("MESSUNG #%d" % measurement_count)
        print("="*70)

        init_hardware()
        init_display()

        print("\n" + "="*70)
        print("SENSORMESSUNGEN")
        print("="*70)

        avg_reading_b, vbat = read_battery_voltage()
        avg_reading_s, vsensor = read_sensor_voltage()
        status = determine_moisture_status(avg_reading_s)

        # Prozent berechnen
        battery_percent = calc_battery_percent(vbat)
        sensor_percent = calc_sensor_percent(avg_reading_s)

        print("\n Ergebnis:")
        print(" - Batterie: %.4fV (ADC: %d, %d%%)" % (vbat, avg_reading_b, battery_percent))
        print(" - Sensor: %.4fV (ADC: %d, %d%%)" % (vsensor, avg_reading_s, sensor_percent))
        print(" - Status: %s" % status)
        print("="*70)

        display_values(vbat, vsensor, status, measurement_count)
        time.sleep(3)

        # Screen 3: Icon-Screen mit automatischer Zone-Erkennung
        print(" → Screen 3: Zeige 40×40 Status-Icon...")
        show_icon_screen(display, avg_reading_s)
        time.sleep(3)

        # MQTT data mit UNTERGRUPPEN
        mqtt_data = {
            "device": {
                "devicename": MQTT_CLIENT_ID,
                "timestamp": int(time.time()),
                "measurement_count": measurement_count,
                "software_version": VERSION
            },
            "battery": {
                "b_voltage": round(vbat, 4),
                "b_adc": avg_reading_b,
                "b_percent": battery_percent
            },
            "sensor": {
                "s_voltage": round(vsensor, 4),
                "s_adc": avg_reading_s,
                "s_percent": sensor_percent,
            },
            "s_moisture_status": status
        }

        if connect_wifi():
            publish_mqtt(mqtt_data)
            time.sleep(1)

        button_pressed = check_button_before_sleep(BUTTON_POLL_TIME)

        if button_pressed:
            print("\n→ Nächste Messung sofort!")
            time.sleep(1)
        else:
            if SLEEP_TIME > 0:
                print("\n→ Speichere Counter und gehe in Sleep...")
                # Counter speichern VOR Sleep!
                save_counter(measurement_count)
                if display:
                    try:
                        display.fill(0)
                        display.show()
                        display.poweroff()
                    except:
                        pass
                gc.collect()
                enter_deep_sleep()
            else:
                print("\n→ Dauerbetrieb: Nächste Messung in 3s...")
                time.sleep(3)

# ============================================================================
# PROGRAMMSTART
# ============================================================================

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nProgramm beendet")
        led_off()
    except Exception as e:
        print("\n\n✗ FEHLER: %s" % str(e))
        import sys
        sys.print_exception(e)
        led_blink(10, 0.1)
    finally:
        if display:
            try:
                display.fill(0)
                display.text(" Stop", 0, 20)
                display.show()
            except:
                pass
        led_off()
