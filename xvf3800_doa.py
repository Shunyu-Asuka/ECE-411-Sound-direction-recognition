import sys
import struct
import time
import math

import usb.core
import usb.util

# 参数表：名字 -> (resid, cmdid, length, 权限, 数据类型)
PARAMETERS = {
    "VERSION":             (48, 0, 3,  "ro", "uint8"),
    "AEC_AZIMUTH_VALUES":  (33, 75, 16, "ro", "radians"),
    "DOA_VALUE":           (20, 18, 4, "ro", "uint16"),
    "REBOOT":              (48, 7, 1,  "wo", "uint8"),
}


class ReSpeaker:
    TIMEOUT = 100000  # USB 超时时间

    def __init__(self, dev):
        self.dev = dev

    def write(self, name, data_list):
        try:
            data = PARAMETERS[name]
        except KeyError:
            return

        if data[3] == "ro":
            raise ValueError('{} is read-only'.format(name))
        if len(data_list) != data[2]:
            raise ValueError('{} value count is not {}'.format(name, data[2]))

        windex = data[0]
        wvalue = data[1]
        data_type = data[4]
        data_cnt = data[2]
        payload = []

        if data_type in ('float', 'radians'):
            for i in range(data_cnt):
                payload += struct.pack(b'f', float(data_list[i]))
        elif data_type in ('char', 'uint8'):
            for i in range(data_cnt):
                payload += data_list[i].to_bytes(1, byteorder='little')
        else:
            for i in range(data_cnt):
                payload += struct.pack(b'i', data_list[i])

        self.dev.ctrl_transfer(
            usb.util.CTRL_OUT
            | usb.util.CTRL_TYPE_VENDOR
            | usb.util.CTRL_RECIPIENT_DEVICE,
            0,
            wvalue,
            windex,
            payload,
            self.TIMEOUT,
        )

    def read(self, name):
        try:
            data = PARAMETERS[name]
        except KeyError:
            return

        resid = data[0]
        cmdid = 0x80 | data[1]  # 读命令 = 0x80 | cmdid
        length = data[2] + 1    # 多 1 字节状态位

        response = self.dev.ctrl_transfer(
            usb.util.CTRL_IN
            | usb.util.CTRL_TYPE_VENDOR
            | usb.util.CTRL_RECIPIENT_DEVICE,
            0,
            cmdid,
            resid,
            length,
            self.TIMEOUT,
        )

        # 不同类型的解析方式
        if data[4] == 'uint8':
            result = response.tolist()
        elif data[4] == 'radians':
            byte_data = response.tobytes()
            num_values = int((length - 1) / 4)
            fmt = '<' + 'f' * num_values
            result = struct.unpack(fmt, byte_data[1:length])
        elif data[4] == 'uint16':
            # 保留原始 uint8 列表，由我们自己拼 uint16
            result = response.tolist()
        else:
            result = response.tolist()

        return result

    def close(self):
        usb.util.dispose_resources(self.dev)


def find():
    """
    尝试多种 PID，或者至少匹配到任意 0x2886 设备
    """
    vids_pids = [
        (0x2886, 0x001A),
        (0x2886, 0x0018),
    ]
    for vid, pid in vids_pids:
        dev = usb.core.find(idVendor=vid, idProduct=pid)
        if dev:
            return ReSpeaker(dev)

    # 兜底：只按 Vendor ID 找
    dev = usb.core.find(idVendor=0x2886)
    if dev:
        return ReSpeaker(dev)

    return None


def main():
    dev = find()
    if not dev:
        print('No XVF3800 device found')
        sys.exit(1)

    # 读一下版本信息
    version = dev.read("VERSION")
    print('VERSION RAW:', version)

    print("开始连续读取 DOA_VALUE 和 AEC_AZIMUTH_VALUES，按 Ctrl+C 结束。\n")

    try:
        while True:
            # 1) 原始 DOA_VALUE
            doa = dev.read("DOA_VALUE")
            print("RAW DOA_VALUE:", doa)

            if doa and len(doa) > 0:
                status = doa[0]

                # 角度拼成 uint16（低 8 位 + 高 8 位）
                angle_raw = None
                if len(doa) >= 3:
                    angle_raw = doa[1] | (doa[2] << 8)
                elif len(doa) >= 2:
                    angle_raw = doa[1]

                speech_flag = doa[3] if len(doa) > 3 else None

                print(
                    f"[DOA_VALUE] status={status}, "
                    f"speech_flag={speech_flag}, "
                    f"angle_raw={angle_raw}"
                )

            # 2) AEC_AZIMUTH_VALUES（弧度 → 角度）
            az_values = dev.read("AEC_AZIMUTH_VALUES")
            if az_values and len(az_values) > 0:
                angle0_deg = az_values[0] * 180.0 / math.pi
                if len(az_values) > 1:
                    angle1_deg = az_values[1] * 180.0 / math.pi
                    print(
                        f"[AZIMUTH] angle0={angle0_deg:.1f}°, "
                        f"angle1={angle1_deg:.1f}°"
                    )
                else:
                    print(f"[AZIMUTH] angle0={angle0_deg:.1f}°")

            print("-" * 40)
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n停止读取，退出程序。")

    finally:
        dev.close()


if __name__ == '__main__':
    main()
