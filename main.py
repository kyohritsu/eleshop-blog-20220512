import machine
import time

class Hardware:
    # GPIO 番号の指定
    PIN_LED_RED = 3         # XIAO: D10
    PIN_LED_GREEN = 4       # XIAO: D9
    PIN_LED_BLUE = 2        # XIAO: D8
    PIN_TRIM_HOT = 26       # XIAO: A0
    PIN_TRIM_COLD = 27      # XIAO: A1
    # PWM 周波数 [Hz]
    PWM_FREQ = 400

    # 各ハードウェアオブジェクトの生成、初期化
    def __init__(self):
        # マルチカラーLEDの各色出力ピン、PWM初期化
        self.led = {}
        self.led['red'] = machine.Pin(self.PIN_LED_RED, machine.Pin.OUT)
        self.led['green'] = machine.Pin(self.PIN_LED_GREEN, machine.Pin.OUT)
        self.led['blue'] = machine.Pin(self.PIN_LED_BLUE, machine.Pin.OUT)
        self.pwm = {}
        for k in self.led.keys():
            self.pwm[k] = machine.PWM(self.led[k])
            self.pwm[k].freq(self.PWM_FREQ)
        self.set_led(0, 0, 0)
        # アナログ入力ピン初期化
        self.trim_hot = machine.ADC(machine.Pin(self.PIN_TRIM_HOT))
        self.trim_cold = machine.ADC(machine.Pin(self.PIN_TRIM_COLD))
        # I2C 初期化
        self.i2c = machine.I2C(1, freq=1000000)

    # マルチカラーLEDの各輝度を指定して出力
    def set_led(self, red, green, blue):
        self.pwm['red'].duty_u16(int(65535 * (1.0 - red)))
        self.pwm['green'].duty_u16(int(65535 * (1.0 - green)))
        self.pwm['blue'].duty_u16(int(65535 * (1.0 - blue)))

    # 半固定抵抗のポジション (0..1) を取得
    def read_trim(self):
        return [a.read_u16() / 65535 for a in [self.trim_cold, self.trim_hot]]

    # DHT20 へ計測開始をトリガ
    def dht20_trigger_measurement(self):
        # 約 10mS 待機
        time.sleep(0.01)
        # DHT20 へコマンドを書き込んで計測開始をトリガ
        self.i2c.writeto(0x38, bytes([0xac, 0x33, 0x00]))

    # DHT20 からデータを取得
    def dht20_get_data(self):
        # 約 80mS 待機
        time.sleep(0.08)
        return self.i2c.readfrom(0x38, 7)

class DHT20:
    # 温湿度センサーDHT20から得られたデータの正当性を検査
    @staticmethod
    def verify(b):
        valid = True
        # 入力リストのサイズチェック
        if len(b) != 7:
            valid = False
        # ステータスビットで、計測が完了しているか確認
        # (第0バイト,第7ビットが0であれば計測完了、データ有効)
        if valid and b[0] & 0x80:
            valid = False
        # CRCチェック
        if valid:
            # CRC-8 (CRC-8/MAXIM) 生成多項式: (x8)+x5+x4+1
            CRC8_POLY = 0b00110001
            # CRC-8 初期値
            crc8 = 0xff
            for j in range(len(b) - 1):
                crc8 ^= b[j]
                for i in range(8):
                    msb = crc8 & 0x80
                    crc8 = (crc8 << 1) & 0xff
                    crc8 ^= CRC8_POLY if msb else 0
            # CRC-8計算値 (第0..5バイト) が格納値 (第6バイト) と一致すればOK
            # (第6バイトを含めて計算した場合は、結果が 0 になればOK)
            valid = crc8 == b[6]
        return valid

    # 温湿度センサーDHT20から得られたデータの生の値から温度を求める
    @staticmethod
    def temperature(b):
        result = None
        # 入力リストのサイズチェック
        if len(b) != 7:
            pass
        else:
            # 0b?---?---   b[0]
            #   |   +----- CAL_ENABLE  校正状態 (1=校正済み、0=未校正)
            #   +--------- BUSY        計測状態 (1=計測中、0=待機中)
            # 0b????????   b[1]
            #   ++++++++-- HUMI<19:12> 温度計測結果 第19..12ビット
            # 0b????????   b[2]
            #   ++++++++-- HUMI<11:4> 温度計測結果 第11..4ビット
            # 0b????xxxx   b[3]
            #   ||||++++-- TEMP<19:16> 温度計測結果 第19..16ビット
            #   ++++------ HUMI<3:0>   湿度計測結果 第3..0ビット
            # 0bxxxxxxxx   b[4]
            #   ++++++++-- TEMP<15:8> 温度計測結果 第15..8ビット
            # 0bxxxxxxxx   b[5]
            #   ++++++++-- TEMP<7:0> 温度計測結果 第7..0ビット
            # 0b????????   b[6]
            #   ++++++++-- CRC  CRC検査データ (CRC-8/MAXIM 生成多項式に基づく)
            result = (
                (((b[3] & 0x0f) << 16) | (b[4] << 8) | b[5]) * 200.0
                / (2 ** 20) - 50.0
            )
        return result

class App:
    # 冷点・温点の基準温度を校正
    # limit_low: 調整温度範囲の下限値 [℃]
    # limit_high: 設定温度範囲の上限値 [℃]
    # ratio_cold: 冷点基準温度の線形比率 (0..1, 半固定抵抗のポジションにて決定)
    # ratio_hot: 温点準温度の線形比率 (0..1, 半固定抵抗のポジションにて決定)
    @staticmethod
    def calibrate_references(limit_low, limit_high, ratio_cold, ratio_hot):
        return [
            limit_low + ratio_cold * (limit_high - limit_low),
            limit_low + ratio_hot * (limit_high - limit_low)
        ]

    # 色相分解能に応じた色相コードの有効範囲を求める
    @staticmethod
    def huecode_max(color_depth_bits):
        return 6 << color_depth_bits

    # 色相コードからRGBの輝度に変換
    @staticmethod
    def huecode_to_rgb(huecode, color_depth_bits):
        r = g = b = None
        # 色相60度=1.0 に正規化した色相値
        hue_norm = huecode / (1 << color_depth_bits)
        # 色相値の区間にて、RGB各輝度を線形比率で計算
        if 0.0 <= hue_norm < 1.0:
            # 赤 (R__) ～ 黄 (RG_)
            r = 1.0
            g = hue_norm
            b = 0.0
        elif 1.0 <= hue_norm < 2.0:
            # 黄 (RG_) ～ 緑 (_G_)
            r = 1.0 - (hue_norm - 1.0)
            g = 1.0
            b = 0.0
        elif 2.0 <= hue_norm < 3.0:
            # 緑 (_G_) ～ シアン (_GB)
            r = 0.0
            g = 1.0
            b = hue_norm - 2.0
        elif 3.0 <= hue_norm < 4.0:
            # シアン (_GB) ～ 青 (__B)
            r = 0.0
            g = 1.0 - (hue_norm - 3.0)
            b = 1.0
        elif 4.0 <= hue_norm < 5.0:
            # 青 (__B) ～ マゼンタ (R_B)
            r = hue_norm - 4.0
            g = 0.0
            b = 1.0
        elif 5.0 <= hue_norm < 6.0:
            # マゼンタ (R_B) ～ 赤 (R__)
            r = 1.0
            g = 0.0
            b = 1.0 - (hue_norm - 5.0)
        else:
            # 定義域外の入力
            r = g = b = 0.0
            pass
        return [r, g, b]

    # 読み取った現在温度と冷点・温点の基準温度より、LEDで表示する色相を求める
    @staticmethod
    def temperature_to_huecode(readout, ref_cold, ref_hot, color_depth_bits):
        result = None
        if ref_cold > ref_hot:
            # 冷点基準と温点基準の高低が逆の場合、消灯
            result = -1
        elif readout <= ref_cold:
            # 冷点基準より低い場合、青色を指定
            result = 4 << color_depth_bits
        elif readout >= ref_hot:
            # 温点基準より低い場合、赤色を指定
            result = 0 << color_depth_bits
        else:
            # 冷点基準と温点基準の間の場合
            # 青色～赤色の間の線形比率で求める。※色相値 (赤=0, 青=4) と温度の
            # 関係が逆のため、比例計算部は符号を反転させる
            result = int(
                (1.0 - (readout - ref_cold) / (ref_hot - ref_cold)) * 4.0
                * (1 << color_depth_bits)
            )
        return result

if __name__ == '__main__':
    # ハードウェア初期化
    hw = Hardware()
    # 温点・冷点 各基準温度の調整温度範囲の下限値 [℃]
    REFERENCE_TEMPERATURE_LIMIT_LOW = -5.0
    # 温点・冷点 各基準温度の調整温度範囲の上限値 [℃]
    REFERENCE_TEMPERATURE_LIMIT_HIGH = 40.0
    # 色相60度あたりの分解能 [bit]
    COLOR_DEPTH_BITS = 10
    while True:
        # 半固定抵抗のポジションを読み取り
        trim_cold, trim_hot = hw.read_trim()
        # 温点・冷点の各基準温度を求める
        ref_cold, ref_hot = App.calibrate_references(
            REFERENCE_TEMPERATURE_LIMIT_LOW,
            REFERENCE_TEMPERATURE_LIMIT_HIGH,
            trim_cold, trim_hot
        )
        # 温度データをセンサーから読み取り
        hw.dht20_trigger_measurement()
        r = hw.dht20_get_data()
        t = DHT20.temperature(r) if DHT20.verify(r) else None
        # 基準温度との関係から、LEDの色相を求めて
        hue = App.temperature_to_huecode(t, ref_cold, ref_hot, COLOR_DEPTH_BITS)
        r, g, b = App.huecode_to_rgb(hue, COLOR_DEPTH_BITS)
        # マルチカラーLEDのPWM出力にセット
        hw.set_led(r, g, b)
        # シリアル出力
        print(f'{'%04d' % int(trim_cold * 9999)}', end='')
        print(f',{'%04d' % int(trim_hot * 9999)}', end='')
        print(f',{'%+05d' % int(ref_cold * 100)}' , end='')
        print(f',{'%+05d' % int(t * 100)}' , end='')
        print(f',{'%+05d' % int(ref_hot * 100)}', end=''),
        print(f',{'%+05d' % hue}')
        #print(f'ref_cold={ref_cold}, ref_hot={ref_hot}')
        #print(f'temperature={t}')
        #print(f'data={''.join(['%02x' % i for i in r])}')
