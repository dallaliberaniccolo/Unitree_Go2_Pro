#!/usr/bin/env python3
"""Driver SPI sperimentale ICM-20948 + AK09916, via I2C master interno.

Verificare prima del campo: registri chip id, assi, segni, magnetometro,
calibrazione, valori fisici. Non è un driver di sicurezza certificato.
"""
import time
import spidev

class ICM20948SPI:
    def __init__(self, bus=0, device=0, speed_hz=1000000, accel_dlpf_cfg=0, gyro_dlpf_cfg=0):
        if accel_dlpf_cfg not in range(8) or gyro_dlpf_cfg not in range(8):
            raise ValueError("Invalid ICM20948 DLPF configuration")
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = speed_hz
        self.spi.mode = 0
        self._gyro_config = (gyro_dlpf_cfg << 3) | 0x03
        self._accel_config = (accel_dlpf_cfg << 3) | 0x05
        self.filter_registers = {}
        try:
            self.reinitialize()
        except Exception:
            self.close()
            raise

    def reinitialize(self):
        """Restore configuration after a detected sensor reset, using the same SPI FD."""
        self.bank = None
        self._bank(0)
        who = self._read(0x00, 1)[0]
        if who != 0xEA:
            raise RuntimeError(f'ICM-20948 WHO_AM_I=0x{who:02X}, atteso 0xEA; controlla SPI')
        self._write(0x06, 0x80)  # reset chip
        time.sleep(0.12)
        self.bank = None
        self._bank(0)
        self._write(0x06, 0x01)  # wake, auto clock
        self._write(0x07, 0x00)  # tutti assi on
        time.sleep(0.05)
        self._bank(2)
        self._write(0x01, self._gyro_config)  # +-500 dps, DLPF enabled
        self._write(0x14, self._accel_config)  # +-8 g, DLPF enabled
        self.filter_registers = {'gyro': self._read(0x01, 1)[0], 'accel': self._read(0x14, 1)[0]}
        if self.filter_registers != {'gyro': self._gyro_config, 'accel': self._accel_config}:
            raise RuntimeError('ICM20948 filter register readback mismatch')
        self._bank(0)
        self._write(0x03, 0x30)  # I2C master enable, host I2C disabled
        self._bank(3)
        self._write(0x01, 0x07)  # internal master clock
        self._bank(0)
        time.sleep(0.02)
        magid = self._ak_read(0x01, 1)[0]
        if magid != 0x09:
            raise RuntimeError(f'AK09916 WIA2=0x{magid:02X}; atteso 0x09. SPI IMU OK ma magnetometro NON pronto')
        self._ak_write(0x32, 0x01)  # magnetometer soft reset
        time.sleep(0.03)
        self._ak_write(0x31, 0x08)  # continuous 100 Hz
        time.sleep(0.05)
        self.validate_configuration()

    def _bank(self, index, force=False):
        if force or self.bank != index:
            self.spi.xfer2([0x7F, (index & 3) << 4])
            self.bank = index

    def validate_configuration(self):
        """Reject samples if a reset/brownout changed the chip's live registers.

        The software bank cache alone cannot survive an asynchronous chip reset.
        WHO_AM_I can remain correct while sleep and all filter settings revert.
        """
        expected = {'who_am_i': 0xEA, 'pwr_mgmt_1': 0x01,
                    'pwr_mgmt_2': 0x00, 'user_ctrl': 0x30,
                    'gyro_config': self._gyro_config, 'accel_config': self._accel_config}
        try:
            self._bank(0, force=True)
            actual = {'who_am_i': self._read(0x00, 1)[0],
                      'pwr_mgmt_1': self._read(0x06, 1)[0],
                      'pwr_mgmt_2': self._read(0x07, 1)[0],
                      'user_ctrl': self._read(0x03, 1)[0]}
            self._bank(2, force=True)
            actual.update(gyro_config=self._read(0x01, 1)[0],
                          accel_config=self._read(0x14, 1)[0])
            self._bank(0, force=True)
            mismatches = [f'{key}=0x{actual[key]:02X} expected=0x{value:02X}'
                          for key, value in expected.items() if actual[key] != value]
            if mismatches:
                raise RuntimeError('ICM20948 configuration_lost: ' + '; '.join(mismatches))
        except (OSError, RuntimeError, IndexError):
            self.bank = None
            raise

    def _write(self, reg, value):
        self.spi.xfer2([reg & 0x7F, value & 255])

    def _read(self, reg, count):
        return self.spi.xfer2([reg | 0x80] + [0] * count)[1:]

    def _ak_transaction(self, reg, write_value=None):
        """Transazione SLV4 singola, evita letture ripetute automatiche di ST2."""
        self._bank(0)
        self._read(0x17, 1)  # clear old read-to-clear DONE/NACK status before EN
        self._bank(3)
        self._write(0x13, 0x0C if write_value is not None else 0x8C)
        self._write(0x14, reg)
        if write_value is not None:
            self._write(0x16, write_value)
        self._write(0x15, 0x80)  # I2C_SLV4_CTRL: EN
        deadline = time.monotonic() + 0.08
        while time.monotonic() < deadline:
            self._bank(0)
            status = self._read(0x17, 1)[0]
            if status & 0x20:
                raise RuntimeError(f'AK09916: arbitration lost (status=0x{status:02X})')
            if status & 0x10:
                raise RuntimeError(f"AK09916: SLV4 NACK (reg=0x{reg:02X}, status=0x{status:02X})")
            if status & 0x40:  # I2C_SLV4_DONE
                if write_value is None:
                    self._bank(3)
                    value = self._read(0x17, 1)[0]  # I2C_SLV4_DI
                    self._bank(0)
                    return value
                return None
            time.sleep(0.0005)
        raise TimeoutError(f"AK09916: SLV4 timeout reg 0x{reg:02X}")

    def _ak_write(self, reg, value):
        self._ak_transaction(reg, write_value=value)

    def _ak_read(self, reg, count):
        if not 1 <= count <= 16:
            raise ValueError('count fuori intervallo')
        return [self._ak_transaction(reg + i) for i in range(count)]

    @staticmethod
    def _s16_be(hi, lo):
        n = (hi << 8) | lo
        return n - 65536 if n >= 32768 else n

    @staticmethod
    def _s16_le(lo, hi):
        return ICM20948SPI._s16_be(hi, lo)

    def read_accelerometer_gyro_data(self):
        self.validate_configuration()
        data = self._read(0x2D, 12)
        if len(data) != 12 or all(value == 0 for value in data) or all(value == 255 for value in data):
            self.bank = None
            raise RuntimeError('ICM20948 invalid all-zero/all-FF accelerometer/gyro block')
        vals = [self._s16_be(data[i], data[i+1]) for i in range(0,12,2)]
        return (*[v/4096.0 for v in vals[:3]], *[v/65.5 for v in vals[3:]])

    def magnetometer_diagnostics(self):
        """Registra informazioni AK09916 per capire perché DRDY non si attiva."""
        who = self._ak_read(0x01, 1)[0]
        mode = self._ak_read(0x31, 1)[0]
        data = self._ak_read(0x10, 9)
        self._bank(0)
        status = self._read(0x17, 1)[0]  # I2C_MST_STATUS
        return (f"WIA2=0x{who:02X} CNTL2=0x{mode:02X} "
                f"ST1=0x{data[0]:02X} ST2=0x{data[8]:02X} "
                f"I2C_MST_STATUS=0x{status:02X} "
                f"raw={[hex(v) for v in data]}")

    def read_magnetometer_data(self):
        """Legge ST1 prima e, SOLO se DRDY=1, HXL..ST2.

        Secondo il datasheet AK09916, leggere dati HXL..TMPS o ST2
        azzera DRDY; quindi NON leggerli durante il polling di ST1.
        """
        deadline = time.monotonic() + 0.35
        last_status = 0
        while time.monotonic() < deadline:
            last_status = self._ak_read(0x10, 1)[0]
            if last_status & 0x01:
                # HXL, HXH, HYL, HYH, HZL, HZH, TMPS, ST2
                data = self._ak_read(0x11, 8)
                if data[7] & 0x08:  # HOFL (overflow magnetico)
                    raise RuntimeError('AK09916: overflow magnetometro (HOFL)')
                values = [self._s16_le(data[i], data[i+1]) for i in (0, 2, 4)]
                return tuple(v * 0.15 for v in values)
            time.sleep(0.005)
        raise RuntimeError(
            'AK09916: nessun dato pronto; ultima ST1=0x%02X; ' % last_status
            + self.magnetometer_diagnostics()
        )

    def close(self):
        self.spi.close()
