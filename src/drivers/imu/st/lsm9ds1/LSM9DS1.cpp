/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "LSM9DS1.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

LSM9DS1::LSM9DS1(I2CSPIBusOption bus_option, int bus, uint32_t device, enum Rotation rotation, int bus_frequency,
		 spi_mode_e spi_mode) :
	SPI(DRV_IMU_DEVTYPE_ST_LSM9DS1_AG, MODULE_NAME, bus, device, spi_mode, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_px4_accel(get_device_id(), rotation),
	_px4_gyro(get_device_id(), rotation)
{
	ConfigureSampleRate(_px4_gyro.get_max_rate_hz());
}

LSM9DS1::~LSM9DS1()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_drdy_missed_perf);
}

int LSM9DS1::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool LSM9DS1::Reset()
{
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void LSM9DS1::exit_and_cleanup()
{
	I2CSPIDriverBase::exit_and_cleanup();
}

void LSM9DS1::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("FIFO empty interval: %d us (%.1f Hz)", _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us);

	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_drdy_missed_perf);
}

int LSM9DS1::probe()
{
	const uint8_t whoami = RegisterRead(Register::WHO_AM_I);

	if (whoami != WHO_AM_I_ID) {
		DEVICE_DEBUG("unexpected WHO_AM_I 0x%02x", whoami);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void LSM9DS1::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// PWR_MGMT_1: Device Reset
		RegisterWrite(Register::CTRL_REG8, CTRL_REG8_BIT::SW_RESET);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(100_ms);
		break;

	case STATE::WAIT_FOR_RESET:
		if ((RegisterRead(Register::WHO_AM_I) == WHO_AM_I_ID)) {

			// Disable I2C, wakeup, and reset digital signal path
			RegisterWrite(Register::CTRL_REG9, CTRL_REG9_BIT::I2C_DISABLE); // set immediately to prevent switching into I2C mode

			// if reset succeeded then configure
			_state = STATE::CONFIGURE;
			ScheduleDelayed(100_ms);

		} else {
			// RESET not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Reset failed, retrying");
				_state = STATE::RESET;
				ScheduleDelayed(100_ms);

			} else {
				PX4_DEBUG("Reset not complete, check again in 10 ms");
				ScheduleDelayed(10_ms);
			}
		}

		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start reading from FIFO
			_state = STATE::FIFO_READ;
			ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);
			FIFOReset();

		} else {
			// CONFIGURE not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Configure failed, resetting");
				_state = STATE::RESET;

			} else {
				PX4_DEBUG("Configure failed, retrying");
			}

			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::FIFO_READ: {
			// always check current FIFO count
			bool success = false;
			// Number of unread words (16-bit axes) stored in FIFO.
			const uint8_t FIFO_SRC = RegisterRead(Register::FIFO_SRC);
			const uint8_t samples = FIFO_SRC & static_cast<uint8_t>(FIFO_SRC_BIT::FSS);

			if (FIFO_SRC & FIFO_SRC_BIT::OVRN) {
				// overflow
				FIFOReset();
				perf_count(_fifo_overflow_perf);

			} else if (samples == 0) {
				perf_count(_fifo_empty_perf);

			} else {
				if (samples > FIFO_MAX_SAMPLES) {
					// not technically an overflow, but more samples than we expected or can publish
					FIFOReset();
					perf_count(_fifo_overflow_perf);

				} else if (samples >= 1) {
					if (FIFORead(now, samples)) {
						success = true;

						if (_failure_count > 0) {
							_failure_count--;
						}
					}
				}
			}

			if (!success) {
				_failure_count++;

				// full reset if things are failing consistently
				if (_failure_count > 10) {
					Reset();
					return;
				}
			}

			if (!success || hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {
				// check configuration registers periodically or immediately following any failure
				if (RegisterCheck(_register_cfg[_checked_register])) {
					_last_config_check_timestamp = now;
					_checked_register = (_checked_register + 1) % size_register_cfg;

				} else {
					// register check failed, force reset
					perf_count(_bad_register_perf);
					Reset();
				}

			} else {
				// periodically update temperature (~1 Hz)
				if (hrt_elapsed_time(&_temperature_update_timestamp) >= 1_s) {
					UpdateTemperature();
					_temperature_update_timestamp = now;
				}
			}
		}

		break;
	}
}

void LSM9DS1::ConfigureSampleRate(int sample_rate)
{
	if (sample_rate == 0) {
		sample_rate = ST_LSM9DS1::LA_ODR;
	}

	// round down to nearest FIFO sample dt
	const float min_interval = FIFO_SAMPLE_DT;
	_fifo_empty_interval_us = math::max(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval, min_interval);

	_fifo_gyro_samples = roundf(math::min((float)_fifo_empty_interval_us / (1e6f / GYRO_RATE), (float)FIFO_MAX_SAMPLES));

	// recompute FIFO empty interval (us) with actual gyro sample limit
	_fifo_empty_interval_us = _fifo_gyro_samples * (1e6f / GYRO_RATE);
}

bool LSM9DS1::Configure()
{
	// first set and clear all configured register bits
	for (const auto &reg_cfg : _register_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	// now check that all are configured
	bool success = true;

	for (const auto &reg_cfg : _register_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	// Gyroscope configuration 2000 degrees/second
	_px4_gyro.set_scale(math::radians(70.f / 1000.f)); // 70 mdps/LSB
	_px4_gyro.set_range(math::radians(2000.f));

	// Accelerometer configuration 16 G range
	_px4_accel.set_scale(0.732f * (CONSTANTS_ONE_G / 1000.f)); // 0.732 mg/LSB
	_px4_accel.set_range(16.f * CONSTANTS_ONE_G);

	return success;
}

bool LSM9DS1::RegisterCheck(const register_config_t &reg_cfg)
{
	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	return success;
}

uint8_t LSM9DS1::RegisterRead(Register reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | READ_BIT;
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

void LSM9DS1::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

void LSM9DS1::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

bool LSM9DS1::FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples)
{
	FIFOTransferBuffer buffer{};
	const size_t transfer_size = math::min(samples * sizeof(FIFO::DATA) + 1, FIFO::SIZE);

	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, transfer_size) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	if (samples > 0) {
		ProcessGyro(timestamp_sample, buffer.f, samples);
		ProcessAccel(timestamp_sample, buffer.f, samples);
		return true;
	}

	return false;
}

void LSM9DS1::FIFOReset()
{
	perf_count(_fifo_reset_perf);

	// FIFO_CTRL: to reset FIFO content, Bypass mode (0) should be selected
	RegisterWrite(Register::FIFO_CTRL, 0);

	// After this reset command, it is possible to restart FIFO mode by writing FIFO_CTRL (2Eh) (FMODE [2:0]) to '001'.
	for (auto &r : _register_cfg) {
		if ((r.reg == Register::CTRL_REG8) || (r.reg == Register::CTRL_REG9) || (r.reg == Register::FIFO_CTRL)) {
			RegisterSetAndClearBits(r.reg, r.set_bits, r.clear_bits);
		}
	}
}

void LSM9DS1::ProcessAccel(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples)
{
	sensor_accel_fifo_s accel{};
	accel.timestamp_sample = timestamp_sample;
	accel.samples = samples;
	accel.dt = FIFO_SAMPLE_DT;

	for (int i = 0; i < samples; i++) {
		const int16_t accel_x = combine(fifo[i].OUT_X_H_XL, fifo[i].OUT_X_L_XL);
		const int16_t accel_y = combine(fifo[i].OUT_Y_H_XL, fifo[i].OUT_Y_L_XL);
		const int16_t accel_z = combine(fifo[i].OUT_Z_H_XL, fifo[i].OUT_Z_L_XL);

		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		accel.x[i] = accel_x;
		accel.y[i] = accel_y;
		accel.z[i] = (accel_z == INT16_MIN) ? INT16_MAX : -accel_z;
	}

	_px4_accel.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				   perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

	_px4_accel.updateFIFO(accel);
}

void LSM9DS1::ProcessGyro(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples)
{
	sensor_gyro_fifo_s gyro{};
	gyro.timestamp_sample = timestamp_sample;
	gyro.samples = samples;
	gyro.dt = FIFO_SAMPLE_DT;

	for (int i = 0; i < samples; i++) {
		const int16_t gyro_x = combine(fifo[i].OUT_X_H_G, fifo[i].OUT_X_L_G);
		const int16_t gyro_y = combine(fifo[i].OUT_Y_H_G, fifo[i].OUT_Y_L_G);
		const int16_t gyro_z = combine(fifo[i].OUT_Z_H_G, fifo[i].OUT_Z_L_G);

		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		gyro.x[i] = gyro_x;
		gyro.y[i] = gyro_y;
		gyro.z[i] = (gyro_z == INT16_MIN) ? INT16_MAX : -gyro_z;
	}

	_px4_gyro.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				  perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

	_px4_gyro.updateFIFO(gyro);
}

void LSM9DS1::UpdateTemperature()
{
	// read current temperature
	uint8_t temperature_buf[3] {};
	temperature_buf[0] = static_cast<uint8_t>(Register::OUT_TEMP_L) | READ_BIT | MS_BIT;

	if (transfer(temperature_buf, temperature_buf, sizeof(temperature_buf)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return;
	}

	// 16 bits in two’s complement format with a sensitivity of 256 LSB/°C. The output zero level corresponds to 25 °C.
	const int16_t OUT_TEMP = combine(temperature_buf[1], temperature_buf[2] & 0x0F);
	const float temperature = (OUT_TEMP / 256.0f) + 25.0f;

	if (PX4_ISFINITE(temperature)) {
		_px4_accel.set_temperature(temperature);
		_px4_gyro.set_temperature(temperature);
	}
}
