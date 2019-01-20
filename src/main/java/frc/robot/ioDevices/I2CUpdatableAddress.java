package frc.robot.iodevices;

/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//https://github.com/FRC-Team-Vern/VL53L0X_Example/blob/master/src/org/usfirst/frc/team5461/robot/sensors/I2CUpdatableAddress.javaz

import java.nio.ByteBuffer;

//import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.I2C;

/**
 * I2C bus interface class.
 *
 * <p>
 * This class is intended to be used by sensor (and other I2C device) drivers.
 * It probably should not be used directly.
 * 
 * @author FRC Team 5461
 */
public class I2CUpdatableAddress extends I2C {

	private Port _port;
	private int _device;

	public I2CUpdatableAddress(Port port, int deviceAddress) {
		super(port, deviceAddress);
		this._port = port;
		this._device = deviceAddress;
	}

	public Port getPort() {
		return _port;
	}

	/**
	 * Read a single bit from an 8-bit device register.
	 *
	 * @param reg Register regAddr to read from
	 * @param bit Bit position to read (0-7)
	 * @return Status of read operation 'transaction aborted ?'
	 */
	public boolean readBitBoolean(int reg, int bit, boolean[] data) {
		byte[] a = new byte[1];
		boolean failed = readBit(reg, bit, a);
		data[0] = (a[0] & 0x01) != 0;
		// if (failed)
		// Robot.robotLogger.log(Logger.INFO, this, "read failed");
		return failed;
	}

	/**
	 * Read a single bit from an 8-bit device register.
	 *
	 * @param reg  Register regAddr to read from
	 * @param bit  Bit position to read (0-7)
	 * @param data Container for single bit value
	 * @return Status of read operation 'transaction aborted ?'
	 */
	public boolean readBit(int reg, int bit, byte[] data) {
		boolean failed = readByte(reg, data);
		data[0] = (byte) (data[0] & (1 << bit));
		// if (failed)
		// Robot.robotLogger.log(Logger.INFO, this, "read failed");
		return failed;
	}

	/**
	 * Read multiple bits from an 8-bit device register.
	 *
	 * @param reg      Register regAddr to read from
	 * @param bitStart First bit position to read (0-7)
	 * @param length   Number of bits to read (not more than 8)
	 * @param data     Container for right-aligned value (i.e. '101' read from any
	 *                 bitStart position will equal 0x05)
	 * @return Status of read operation 'transaction aborted ?'
	 */
	public boolean readBits(int reg, int bitStart, int length, byte[] data) {
		byte[] b = new byte[1];
		if (readByte(reg, b)) {
			// Robot.robotLogger.log(Logger.INFO, this, "read failed");
			return true; // transaction failed
		}
		byte mask = (byte) (((1 << length) - 1) << (bitStart - length + 1));
		b[0] &= mask;
		b[0] >>= (byte) (bitStart - length + 1);
		data[0] = b[0];
		return false;
	}

	public boolean readByte(int registerAddress, byte[] data) {
		boolean failed = read(registerAddress, 1, data);
		data[0] = (byte) (data[0] & 0xff);
		// if (failed)
		// Robot.robotLogger.log(Logger.INFO, this, "read failed");
		return failed;
	}

	public boolean readShort(int registerAddress, short[] data) {
		ByteBuffer buffer = ByteBuffer.allocate(2);
		boolean failed = read(registerAddress, 2, buffer);
		data[0] = buffer.getShort();
		// if (failed)
		// Robot.robotLogger.log(Logger.INFO, this, "read failed");
		return failed;
	}

	/**
	 * write a single bit in an 8-bit device register.
	 *
	 * @param reg   Register regAddr to write to
	 * @param bit   Bit position to write (0-7)
	 * @param value New bit value to write
	 * @return 'transaction aborted ?'
	 */
	public synchronized final boolean writeBit(int reg, int bit, byte value) {
		return writeBit(reg, bit, value != 0);
	}

	/**
	 * write a single bit in an 8-bit device register.
	 *
	 * @param reg   Register regAddr to write to
	 * @param bit   Bit position to write (0-7)
	 * @param value New bit value to write
	 * @return 'transaction aborted ?'
	 */
	public boolean writeBit(int reg, int bit, boolean value) {
		byte[] b = new byte[1];
		if (readByte(reg, b)) {
			// Robot.robotLogger.log(Logger.INFO, this, "read failed");
			return true; // transaction failed
		}
		b[0] = (byte) (value ? (b[0] | (1 << bit)) : (b[0] & ~(1 << bit)));
		return writeByte(reg, b[0]);
	}

	/**
	 * Write multiple bits in an 8-bit device register.
	 *
	 * @param reg      Register regAddr to write to
	 * @param bitStart First bit position to write (0-7)
	 * @param length   Number of bits to write (not more than 8)
	 * @param data     Right-aligned value to write
	 * @return 'transaction aborted ?'
	 */
	public boolean writeBits(int reg, int bitStart, int length, byte data) {
		byte[] b = new byte[1];
		if (readByte(reg, b)) {
			// Robot.robotLogger.log(Logger.INFO, this, "read failed");
			return true; // transaction failed
		}
		byte mask = (byte) (((1 << length) - 1) << (bitStart - length + 1));
		data <<= (bitStart - length + 1);
		data &= mask;
		b[0] &= ~(mask);
		b[0] |= data;
		return writeByte(reg, b[0]);
	}

	public boolean writeByte(int registerAddress, byte data) {
		// simple passthru to maintain API consistency with 'read' methods
		boolean failed = write(registerAddress, (((int) data) & 0xff));
		// if (failed)
		// Robot.robotLogger.log(Logger.INFO, this, "write failed");
		return failed;
	}

	public boolean writeByte(int registerAddress, int data) {
		// simple passthru to maintain API consistency with 'read' methods
		boolean failed = write(registerAddress, (data & 0xff));
		// if (failed)
		// Robot.robotLogger.log(Logger.INFO, this, "write failed");
		return failed;
	}

	public boolean writeShort(int registerAddress, int data) {
		// the following guarantees correct byte order regardless of big/little endian
		ByteBuffer registerWithDataToSendBuffer = ByteBuffer.allocateDirect(3);
		registerWithDataToSendBuffer.put((byte) registerAddress);
		registerWithDataToSendBuffer.putShort(1, (short) data);
		boolean failed = writeBulk(registerWithDataToSendBuffer, 3);
		// if(failed)
		// Robot.robotLogger.log(Logger.INFO, this, "write failed");
		return failed;
	}

	public final int setAddress(int deviceCommandSetAddress, int new_address) {
		// Robot.robotLogger.log(Logger.DEBUG, this, "enter");
		// NOTICE: CHANGING THE ADDRESS IS NOT STORED IN NON-VOLATILE MEMORY
		// POWER CYCLING THE DEVICE REVERTS ADDRESS BACK TO its default
		// Field field = I2C.class.getDeclaredField("m_deviceAddress");
		// field.setAccessible(true);
		//
		// int deviceAddress = (int) field.get(this);

		// changing an I2C device address, when there are more than one device
		// defaulting to the same address
		// requires that all other devices on the same address be disabled first
		// this is typically done with a digital IO signal pin - DigitalOutput(pin);
		// /* for all other devices */ DigitalOutput.set(false); /* for current device
		// */ DigitalOutput.set(true); setAddress(new_address);
		// example of wiring:
		// https://raw.githubusercontent.com/johnbryanmoore/VL53L0X_rasp_python/master/VL53L0X_Mutli_Rpi3_bb.jpg
		// example of digital IO control code:
		// https://github.com/FRC-Team-Vern/VL53L0X_Example/blob/master/src/org/usfirst/frc/team5461/robot/sensors/VL53L0XSensors.java

		// Robot.robotLogger.log(Logger.DEBUG, this, "setting address to 0x" +
		// Integer.toHexString(new_address));

		if (this._device == new_address) {
			// Robot.robotLogger.log(Logger.DEBUG, this, "exit - no address change
			// required");
			return this._device;
		}
		// Device addresses cannot go higher than 127
		if (new_address > 127) {
			// Robot.robotLogger.log(Logger.ERROR, this, "address out of range");
			return this._device;
		}

		boolean success = write(deviceCommandSetAddress, new_address & 0x7F);
		if (success) // {
			this._device = new_address;
		// } else
		// Robot.robotLogger.log(Logger.ERROR, this, "failed to change address");

		// Robot.robotLogger.log(Logger.DEBUG, this, "exit");
		return this._device;
	}

	public final int getAddressFromDevice(int deviceCommandGetAddress) {
		// Robot.robotLogger.log(Logger.DEBUG, this, "enter");
		byte[] val = new byte[1];
		readByte(deviceCommandGetAddress, val);
		// Robot.robotLogger.log(Logger.DEBUG, this, "exit");
		return val[0];
	}

}