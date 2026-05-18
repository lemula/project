import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Calibrate each potentiometer with two positions
STEER_V1 = 0.748
STEER_V2 = 1.83
STEER_MIN_ANGLE = 40
STEER_MAX_ANGLE = -40

BOOM_V1 = 2.296
BOOM_V2 = 3.065
BOOM_MIN_ANGLE = 80
BOOM_MAX_ANGLE = 0

BUCKET_V1 = 2.000
BUCKET_V2 = 3.111
BUCKET_MIN_ANGLE = 20
BUCKET_MAX_ANGLE = -36


def voltage_to_angle(v: float, v_min: float, v_max: float, min_angle: float, max_angle: float) -> float:
    if v < v_min:
        v = v_min
    if v > v_max:
        v = v_max
    return (v - v_min) / (v_max - v_min) * (max_angle - min_angle) + min_angle


class ThreeAxisPotReader:
    def __init__(self, i2c=None, address=0x48):
        if i2c is None:
            i2c = busio.I2C(board.SCL, board.SDA)
        ads = ADS.ADS1115(i2c, address=address)
        ads.gain = 1
        ads.data_rate = 128
        self.steer_chan = AnalogIn(ads, 0)   # A0 vs GND
        self.boom_chan = AnalogIn(ads, 1)  # A1 vs GND
        self.bucket_chan = AnalogIn(ads, 2)   # A2 vs GND

    def read(self):
        steer_v = self.steer_chan.voltage
        boom_v = self.boom_chan.voltage
        bucket_v = self.bucket_chan.voltage

        steer_angle = voltage_to_angle(steer_v, STEER_V1, STEER_V2, STEER_MIN_ANGLE, STEER_MAX_ANGLE)
        boom_angle = voltage_to_angle(
            boom_v, BOOM_V1, BOOM_V2, BOOM_MIN_ANGLE, BOOM_MAX_ANGLE
        )
        bucket_angle = voltage_to_angle(
            bucket_v, BUCKET_V1, BUCKET_V2, BUCKET_MIN_ANGLE, BUCKET_MAX_ANGLE
        )

        return {
            "steer_v": steer_v,
            "steer_angle": steer_angle,
            "boom_v": boom_v,
            "boom_angle": boom_angle,
            "bucket_v": bucket_v,
            "bucket_angle": bucket_angle,
        }


if __name__ == "__main__":
    reader = ThreeAxisPotReader()
    print("Running... Reading angle from calibrated min/max.")
    while True:
        data = reader.read()
        print(
            "steer: V={:.3f} angle={:.1f} | boom: V={:.3f} angle={:.1f} | bucket: V={:.3f} angle={:.1f}".format(
                data["steer_v"],
                data["steer_angle"],
                data["boom_v"],
                data["boom_angle"],
                data["bucket_v"],
                data["bucket_angle"],
            )
        )
        time.sleep(0.2)