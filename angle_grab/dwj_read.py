import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Calibrate each potentiometer with two positions
BIG_ARM_V_MIN = 1.124
BIG_ARM_V_MAX = 2.236
BIG_ARM_MIN_ANGLE = 0
BIG_ARM_MAX_ANGLE = 90

SMALL_ARM_V_MIN = 0.04
SMALL_ARM_V_MAX = 1.168
SMALL_ARM_MIN_ANGLE = 150
SMALL_ARM_MAX_ANGLE = 50

BUCKET_V_MIN = 1.74
BUCKET_V_MAX = 2.732
BUCKET_MIN_ANGLE = 65
BUCKET_MAX_ANGLE = 220


def voltage_to_angle(v: float, v_min: float, v_max: float, min_angle: float, max_angle: float) -> float:
    if v < v_min:
        v = v_min
    if v > v_max:
        v = v_max
    return (v - v_min) / (v_max - v_min) * (max_angle - min_angle) + min_angle


class PotentiometerReader:
    def __init__(self, i2c=None, address=0x48):
        if i2c is None:
            i2c = busio.I2C(board.SCL, board.SDA)
        ads = ADS.ADS1115(i2c, address=address)
        ads.gain = 1  # ±4.096V range (good for 0-3.3V input)
        ads.data_rate = 128
        self._big_arm_chan = AnalogIn(ads, 0)   # A0 vs GND
        self._small_arm_chan = AnalogIn(ads, 1)  # A1 vs GND
        self._bucket_chan = AnalogIn(ads, 2)   # A2 vs GND

    def read(self):
        big_v = self._big_arm_chan.voltage
        small_v = self._small_arm_chan.voltage
        bucket_v = self._bucket_chan.voltage

        big_angle = voltage_to_angle(big_v, BIG_ARM_V_MIN, BIG_ARM_V_MAX, BIG_ARM_MIN_ANGLE, BIG_ARM_MAX_ANGLE)
        small_angle = voltage_to_angle(
            small_v, SMALL_ARM_V_MIN, SMALL_ARM_V_MAX, SMALL_ARM_MIN_ANGLE, SMALL_ARM_MAX_ANGLE
        )
        bucket_angle = voltage_to_angle(
            bucket_v, BUCKET_V_MIN, BUCKET_V_MAX, BUCKET_MIN_ANGLE, BUCKET_MAX_ANGLE
        )

        return {
            "big_arm_v": big_v,
            "big_arm_angle": big_angle,
            "small_arm_v": small_v,
            "small_arm_angle": small_angle,
            "bucket_v": bucket_v,
            "bucket_angle": bucket_angle,
        }


if __name__ == "__main__":
    reader = PotentiometerReader()
    print("Running... Reading angle from calibrated min/max.")
    while True:
        data = reader.read()
        print(
            "big_arm: V={:.3f} angle={:.1f} | small_arm: V={:.3f} angle={:.1f} | bucket: V={:.3f} angle={:.1f}".format(
                data["big_arm_v"],
                data["big_arm_angle"],
                data["small_arm_v"],
                data["small_arm_angle"],
                data["bucket_v"],
                data["bucket_angle"],
            )
        )
        time.sleep(0.2)

