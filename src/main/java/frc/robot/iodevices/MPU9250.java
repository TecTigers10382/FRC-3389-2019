package frc.robot.iodevices;

public class MPU9250 extends I2CUpdatableAddress {

	/*
	 * -----------------------------------------------------------------------
	 * DEFAULT VALUES
	 * -----------------------------------------------------------------------
	 */

	/**
	 * Default address of the MPU6050 device.
	 */
	public static final int DEFAULT_MPU9250_ADDRESS = 0x68;

	/**
	 * Default value for the digital low pass filter (DLPF) setting for both
	 * gyroscope and accelerometer.
	 */
	public static final int DEFAULT_DLPF_CFG = 0x06;

	/**
	 * Default value for the sample rate divider.
	 */
	public static final int DEFAULT_SMPLRT_DIV = 0x00;

	/**
	 * Coefficient to convert an angle value from radians to degrees.
	 */
	public static final double RADIAN_TO_DEGREE = 180. / Math.PI;

	/**
	 * It is impossible to calculate an angle for the z axis from the accelerometer.
	 */
	private static final double ACCEL_Z_ANGLE = 0;

	/*
	 * -----------------------------------------------------------------------
	 * REGISTERS ADDRESSES
	 * -----------------------------------------------------------------------
	 */

	/**
	 * <b>[datasheet 2 - p.11]</b> Sample Rate Divider
	 * <p>
	 * This register specifies the divider from the gyroscope output rate used to
	 * generate the Sample Rate for the MPU-60X0.
	 * </p>
	 */
	public static final int MPU9250_REG_ADDR_SMPRT_DIV = 0x19; // 25

	/**
	 * <b>[datasheet 2 - p.13]</b> Configuration
	 * <p>
	 * This register configures the external Frame Synchronization (FSYNC) pin
	 * sampling and the Digital Low Pass Filter (DLPF) setting for both the
	 * gyroscopes and accelerometers.
	 * </p>
	 */
	public static final int MPU9250_REG_ADDR_CONFIG = 0x1A; // 26

	/**
	 * <b>[datasheet 2 - p.14]</b> Gyroscope Configuration
	 * <p>
	 * This register is used to trigger gyroscope self-test and configure the
	 * gyroscopes full scale range
	 * </p>
	 */
	public static final int MPU9250_REG_ADDR_GYRO_CONFIG = 0x1B; // 27

	/**
	 * <b>[datasheet 2 - p.15]</b> Accelerometer Configuration
	 * <p>
	 * This register is used to trigger accelerometer self test and configure the
	 * accelerometer full scale range. This register also configures the Digital
	 * High Pass Filter (DHPF).
	 * </p>
	 */
	public static final int MPU9250_REG_ADDR_ACCEL_CONFIG = 0x1C; // 28

	/**
	 * <b>[datasheet 2 - p.27]</b> Interrupt Enable
	 * <p>
	 * This register enables interrupt generation by interrupt sources.
	 * </p>
	 */
	public static final int MPU9250_REG_ADDR_INT_ENABLE = 0x1A; // 56

	/**
	 * <b>[datasheet 2 - p.40]</b> Power Management 1
	 * <p>
	 * This register allows the user to configure the power mode and clock source.
	 * It also provides a bit for resetting the entire device, and a bit for
	 * disabling the temperature sensor.
	 * </p>
	 */
	public static final int MPU9250_REG_ADDR_PWR_MGMT_1 = 0x6B; // 107

	/**
	 * <b>[datasheet 2 - p.42]</b> Power Management 2
	 * <p>
	 * This register allows the user to configure the frequency of wake-ups in
	 * Accelerometer Only Low Power Mode. This register also allows the user to put
	 * individual axes of the accelerometer and gyroscope into standby mode.
	 * </p>
	 */
	public static final int MPU9250_REG_ADDR_PWR_MGMT_2 = 0x6C; // 108

	/**
	 * <b>[datasheet 2 - p.29]</b> Accelerometer Measurements
	 * <p>
	 * These registers store the most recent accelerometer measurements.
	 * </p>
	 * 
	 * @see #MPU9250_REG_ADDR_ACCEL_XOUT_L
	 * @see #MPU9250_REG_ADDR_ACCEL_YOUT_H
	 * @see #MPU9250_REG_ADDR_ACCEL_YOUT_L
	 * @see #MPU9250_REG_ADDR_ACCEL_ZOUT_H
	 * @see #MPU9250_REG_ADDR_ACCEL_ZOUT_L
	 */
	public static final int MPU9250_REG_ADDR_ACCEL_XOUT_H = 0x3B; // 59

	/**
	 * <b>[datasheet 2 - p.29]</b> Accelerometer Measurements
	 * <p>
	 * These registers store the most recent accelerometer measurements.
	 * </p>
	 * 
	 * @see #MPU9250_REG_ADDR_ACCEL_XOUT_H
	 * @see #MPU9250_REG_ADDR_ACCEL_YOUT_H
	 * @see #MPU9250_REG_ADDR_ACCEL_YOUT_L
	 * @see #MPU9250_REG_ADDR_ACCEL_ZOUT_H
	 * @see #MPU9250_REG_ADDR_ACCEL_ZOUT_L
	 */
	public static final int MPU9250_REG_ADDR_ACCEL_XOUT_L = 0x3C; // 60

	/**
	 * <b>[datasheet 2 - p.29]</b> Accelerometer Measurements
	 * <p>
	 * These registers store the most recent accelerometer measurements.
	 * </p>
	 * 
	 * @see #MPU9250_REG_ADDR_ACCEL_XOUT_H
	 * @see #MPU9250_REG_ADDR_ACCEL_XOUT_L
	 * @see #MPU9250_REG_ADDR_ACCEL_YOUT_L
	 * @see #MPU9250_REG_ADDR_ACCEL_ZOUT_H
	 * @see #MPU9250_REG_ADDR_ACCEL_ZOUT_L
	 */
	public static final int MPU9250_REG_ADDR_ACCEL_YOUT_H = 0x3D; // 61

	/**
	 * <b>[datasheet 2 - p.29]</b> Accelerometer Measurements
	 * <p>
	 * These registers store the most recent accelerometer measurements.
	 * </p>
	 * 
	 * @see #MPU9250_REG_ADDR_ACCEL_XOUT_H
	 * @see #MPU9250_REG_ADDR_ACCEL_XOUT_L
	 * @see #MPU9250_REG_ADDR_ACCEL_YOUT_H
	 * @see #MPU9250_REG_ADDR_ACCEL_ZOUT_H
	 * @see #MPU9250_REG_ADDR_ACCEL_ZOUT_L
	 */
	public static final int MPU9250_REG_ADDR_ACCEL_YOUT_L = 0x3E; // 62

	/**
	 * <b>[datasheet 2 - p.29]</b> Accelerometer Measurements
	 * <p>
	 * These registers store the most recent accelerometer measurements.
	 * </p>
	 * 
	 * @see #MPU9250_REG_ADDR_ACCEL_XOUT_H
	 * @see #MPU9250_REG_ADDR_ACCEL_XOUT_L
	 * @see #MPU9250_REG_ADDR_ACCEL_YOUT_H
	 * @see #MPU9250_REG_ADDR_ACCEL_YOUT_L
	 * @see #MPU9250_REG_ADDR_ACCEL_ZOUT_L
	 */
	public static final int MPU9250_REG_ADDR_ACCEL_ZOUT_H = 0x3F; // 63

	/**
	 * <b>[datasheet 2 - p.29]</b> Accelerometer Measurements
	 * <p>
	 * These registers store the most recent accelerometer measurements.
	 * </p>
	 * 
	 * @see #MPU9250_REG_ADDR_ACCEL_XOUT_H
	 * @see #MPU9250_REG_ADDR_ACCEL_XOUT_L
	 * @see #MPU9250_REG_ADDR_ACCEL_YOUT_H
	 * @see #MPU9250_REG_ADDR_ACCEL_YOUT_L
	 * @see #MPU9250_REG_ADDR_ACCEL_ZOUT_H
	 */
	public static final int MPU9250_REG_ADDR_ACCEL_ZOUT_L = 0x40; // 64

	/**
	 * <b>[datasheet 2 - p.30]</b> Temperature Measurement
	 * <p>
	 * These registers store the most recent temperature sensor measurement.
	 * </p>
	 * 
	 * @see #MPU9250_REG_ADDR_TEMP_OUT_L
	 */
	public static final int MPU9250_REG_ADDR_TEMP_OUT_H = 0x41; // 65

	/**
	 * <b>[datasheet 2 - p.30]</b> Temperature Measurement
	 * <p>
	 * These registers store the most recent temperature sensor measurement.
	 * </p>
	 * 
	 * @see #MPU9250_REG_ADDR_TEMP_OUT_H
	 */
	public static final int MPU9250_REG_ADDR_TEMP_OUT_L = 0x42; // 66

	/**
	 * <b>[datasheet 2 - p.31]</b> Gyroscope Measurements
	 * <p>
	 * These registers store the most recent gyroscope measurements.
	 * </p>
	 * 
	 * @see #MPU9250_REG_ADDR_GYRO_XOUT_L
	 * @see #MPU9250_REG_ADDR_GYRO_YOUT_H
	 * @see #MPU9250_REG_ADDR_GYRO_YOUT_L
	 * @see #MPU9250_REG_ADDR_GYRO_ZOUT_H
	 * @see #MPU9250_REG_ADDR_GYRO_ZOUT_L
	 */
	public static final int MPU9250_REG_ADDR_GYRO_XOUT_H = 0x43; // 67

	/**
	 * <b>[datasheet 2 - p.31]</b> Gyroscope Measurements
	 * <p>
	 * These registers store the most recent gyroscope measurements.
	 * </p>
	 * 
	 * @see #MPU9250_REG_ADDR_GYRO_XOUT_H
	 * @see #MPU9250_REG_ADDR_GYRO_YOUT_H
	 * @see #MPU9250_REG_ADDR_GYRO_YOUT_L
	 * @see #MPU9250_REG_ADDR_GYRO_ZOUT_H
	 * @see #MPU9250_REG_ADDR_GYRO_ZOUT_L
	 */
	public static final int MPU9250_REG_ADDR_GYRO_XOUT_L = 0x44; // 68

	/**
	 * <b>[datasheet 2 - p.31]</b> Gyroscope Measurements
	 * <p>
	 * These registers store the most recent gyroscope measurements.
	 * </p>
	 * 
	 * @see #MPU9250_REG_ADDR_GYRO_XOUT_H
	 * @see #MPU9250_REG_ADDR_GYRO_XOUT_L
	 * @see #MPU9250_REG_ADDR_GYRO_YOUT_L
	 * @see #MPU9250_REG_ADDR_GYRO_ZOUT_H
	 * @see #MPU9250_REG_ADDR_GYRO_ZOUT_L
	 */
	public static final int MPU9250_REG_ADDR_GYRO_YOUT_H = 0x45; // 69

	/**
	 * <b>[datasheet 2 - p.31]</b> Gyroscope Measurements
	 * <p>
	 * These registers store the most recent gyroscope measurements.
	 * </p>
	 * 
	 * @see #MPU9250_REG_ADDR_GYRO_XOUT_H
	 * @see #MPU9250_REG_ADDR_GYRO_XOUT_L
	 * @see #MPU9250_REG_ADDR_GYRO_YOUT_H
	 * @see #MPU9250_REG_ADDR_GYRO_ZOUT_H
	 * @see #MPU9250_REG_ADDR_GYRO_ZOUT_L
	 */
	public static final int MPU9250_REG_ADDR_GYRO_YOUT_L = 0x46; // 70

	/**
	 * <b>[datasheet 2 - p.31]</b> Gyroscope Measurements
	 * <p>
	 * These registers store the most recent gyroscope measurements.
	 * </p>
	 * 
	 * @see #MPU9250_REG_ADDR_GYRO_XOUT_H
	 * @see #MPU9250_REG_ADDR_GYRO_XOUT_L
	 * @see #MPU9250_REG_ADDR_GYRO_YOUT_H
	 * @see #MPU9250_REG_ADDR_GYRO_YOUT_L
	 * @see #MPU9250_REG_ADDR_GYRO_ZOUT_L
	 */
	public static final int MPU9250_REG_ADDR_GYRO_ZOUT_H = 0x47; // 71

	/**
	 * <b>[datasheet 2 - p.31]</b> Gyroscope Measurements
	 * <p>
	 * These registers store the most recent gyroscope measurements.
	 * </p>
	 * 
	 * @see #MPU9250_REG_ADDR_GYRO_XOUT_H
	 * @see #MPU9250_REG_ADDR_GYRO_XOUT_L
	 * @see #MPU9250_REG_ADDR_GYRO_YOUT_H
	 * @see #MPU9250_REG_ADDR_GYRO_YOUT_L
	 * @see #MPU9250_REG_ADDR_GYRO_ZOUT_H
	 */
	public static final int MPU9250_REG_ADDR_GYRO_ZOUT_L = 0x48; // 72

	/*
	 * -----------------------------------------------------------------------
	 * VARIABLES
	 * -----------------------------------------------------------------------
	 */

	/**
	 * Value used for the DLPF config.
	 */
	private int dlpfCfg;

	/**
	 * Value used for the sample rate divider.
	 */
	private int smplrtDiv;

	/**
	 * Sensisitivty of the measures from the accelerometer. Used to convert
	 * accelerometer values.
	 */
	private double accelLSBSensitivity;

	/**
	 * Sensitivity of the measures from the gyroscope. Used to convert gyroscope
	 * values to degrees/sec.
	 */
	private double gyroLSBSensitivity;

	private Thread updatingThread = null;
	private boolean updatingThreadStopped = true;
	private long lastUpdateTime = 0;

	// ACCELEROMETER

	/**
	 * Last acceleration value, in g, retrieved from the accelerometer, for the x
	 * axis.
	 * <p>
	 * <i>(using the updating thread)</i>
	 * </p>
	 */
	private double accelAccelerationX = 0.;

	/**
	 * Last acceleration value, in g, retrieved from the accelerometer, for the y
	 * axis.
	 * <p>
	 * <i>(using the updating thread)</i>
	 * </p>
	 */
	private double accelAccelerationY = 0.;

	/**
	 * Last acceleration value, in g, retrieved from the accelerometer, for the z
	 * axis.
	 * <p>
	 * <i>(using the updating thread)</i>
	 * </p>
	 */
	private double accelAccelerationZ = 0.;

	/**
	 * Last angle value, in degrees, retrieved from the accelerometer, for the x
	 * axis.
	 * <p>
	 * <i>(using the updating thread)</i>
	 * </p>
	 */
	private double accelAngleX = 0.;

	/**
	 * Last angle value, in degrees, retrieved from the accelerometer, for the y
	 * axis.
	 * <p>
	 * <i>(using the updating thread)</i>
	 * </p>
	 */
	private double accelAngleY = 0.;

	/**
	 * Last angle value, in degrees, retrieved from the accelerometer, for the z
	 * axis.
	 * <p>
	 * <i>(using the updating thread)</i>
	 * </p>
	 */
	private double accelAngleZ = 0.;

	// GYROSCOPE

	/**
	 * Last angular speed value, in degrees/sec, retrieved from the gyroscope, for
	 * the x axis.
	 * <p>
	 * <i>(using the updating thread)</i>
	 * </p>
	 */
	private double gyroAngularSpeedX = 0.;

	/**
	 * Last angular speed value, in degrees/sec, retrieved from the gyroscope, for
	 * the y axis.
	 * <p>
	 * <i>(using the updating thread)</i>
	 * </p>
	 */
	private double gyroAngularSpeedY = 0.;

	/**
	 * Last angular speed value, in degrees/sec, retrieved from the gyroscope, for
	 * the z axis.
	 * <p>
	 * <i>(using the updating thread)</i>
	 * </p>
	 */
	private double gyroAngularSpeedZ = 0.;

	/**
	 * Last angle value, in degrees, calculated from the gyroscope, for the x axis.
	 * <p>
	 * <i>(using the updating thread)</i>
	 * </p>
	 */
	private double gyroAngleX = 0.;

	/**
	 * Last angle value, in degrees, calculated from the gyroscope, for the y axis.
	 * <p>
	 * <i>(using the updating thread)</i>
	 * </p>
	 */
	private double gyroAngleY = 0.;

	/**
	 * Last angle value, in degrees, calculated from the gyroscope, for the z axis.
	 * <p>
	 * <i>(using the updating thread)</i>
	 * </p>
	 */
	private double gyroAngleZ = 0.;

	/**
	 * Calculated offset for the angular speed from the gyroscope, for the x axis.
	 */
	private double gyroAngularSpeedOffsetX = 0.;

	/**
	 * Calculated offset for the angular speed from the gyroscope, for the y axis.
	 */
	private double gyroAngularSpeedOffsetY = 0.;

	/**
	 * Calculated offset for the angular speed from the gyroscope, for the z axis.
	 */
	private double gyroAngularSpeedOffsetZ = 0.;

	// FILTERED

	/**
	 * Last angle value, in degrees, calculated from the accelerometer and the
	 * gyroscope, for the x axis.
	 * <p>
	 * <i>(using the updating thread)</i>
	 * </p>
	 */
	private double filteredAngleX = 0.;

	/**
	 * Last angle value, in degrees, calculated from the accelerometer and the
	 * gyroscope, for the y axis.
	 * <p>
	 * <i>(using the updating thread)</i>
	 * </p>
	 */
	private double filteredAngleY = 0.;

	/**
	 * Last angle value, in degrees, calculated from the accelerometer and the
	 * gyroscope, for the z axis.
	 * <p>
	 * <i>(using the updating thread)</i>
	 * </p>
	 */
	private double filteredAngleZ = 0.;

	/*
	 * -----------------------------------------------------------------------
	 * CONSTRUCTORS
	 * -----------------------------------------------------------------------
	 */

	/**
	 * Constructor for a new MPU6050 using the default i2c address and the default
	 * value for the DLPF setting.
	 * 
	 * @see #DEFAULT_MPU9250_ADDRESS
	 * @see #DEFAULT_DLPF_CFG
	 */
	public MPU9250() {
		this(DEFAULT_MPU9250_ADDRESS, DEFAULT_DLPF_CFG, DEFAULT_SMPLRT_DIV);
	}

	/**
	 * Constructor for a new MPU6050 using a specific i2c address and a specific
	 * value for the DLPF setting.
	 * 
	 * @see #DEFAULT_MPU9250_ADDRESS
	 * @see #DEFAULT_DLPF_CFG
	 * @param i2cAddress the i2c address of the MPU6050.
	 * @param dlpfCfg    the value of the DLPF setting.
	 * @param smplrtDiv  the value of the sample rate divider.
	 */
	public MPU9250(int i2cAddress, int dlpfCfg, int smplrtDiv) {
		super(Port.kMXP, i2cAddress);
		this.dlpfCfg = dlpfCfg;
		this.smplrtDiv = smplrtDiv;

		// 1. waking up the MPU6050 (0x00 = 0000 0000) as it starts in sleep mode.
		updateRegisterValue(MPU9250_REG_ADDR_PWR_MGMT_1, 0x00);

		// 2. sample rate divider
		// The sensor register output, FIFO output, and DMP sampling are all based on
		// the Sample Rate.
		// The Sample Rate is generated by dividing the gyroscope output rate by
		// SMPLRT_DIV:
		// Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
		// where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or
		// 7),
		// and 1kHz when the DLPF is enabled (see register 26).
		// SMPLRT_DIV set the rate to the default value : Sample Rate = Gyroscope Rate.
		updateRegisterValue(MPU9250_REG_ADDR_SMPRT_DIV, smplrtDiv);

		// 3. This register configures the external Frame Synchronization (FSYNC)
		// pin sampling and the Digital Low Pass Filter (DLPF) setting for both
		// the gyroscopes and accelerometers.
		setDLPFConfig(dlpfCfg);

		// 4. Gyroscope configuration
		// FS_SEL selects the full scale range of the gyroscope outputs.
		byte fsSel = 0 << 3; // FS_SEL +- 250 deg/s
		gyroLSBSensitivity = 131.; // cfr [datasheet 2 - p.31]
		updateRegisterValue(MPU9250_REG_ADDR_GYRO_CONFIG, fsSel);

		// 5. Accelerometer configuration [datasheet 2 - p.29]
		// AFS_SEL 0 is +/- 2g with sensitivity factor of 16384
		// AFS_SEL 1 is +/- 4g with sensitivity factor of 8192
		// AFS_SEL 2 is +/- 8g with sensitivity factor of 4096
		// AFS_SEL 3 is +/- 16g with sensitivity factor of 2048
		// NOTE: changing AFS_SEL to 1 caused more drift not sure why
		byte afsSel = 0;
		accelLSBSensitivity = 16384.;
		updateRegisterValue(MPU9250_REG_ADDR_ACCEL_CONFIG, afsSel);

		// 6. Disable interrupts
		updateRegisterValue(MPU9250_REG_ADDR_INT_ENABLE, 0x00);

		// 7. Disable standby mode
		updateRegisterValue(MPU9250_REG_ADDR_PWR_MGMT_2, 0x00);

		calibrateSensors();
	}

	/*
	 * -----------------------------------------------------------------------
	 * METHODS
	 * -----------------------------------------------------------------------
	 */

	/**
	 * Returns the Sample Rate of the MPU6050.
	 * 
	 * [datasheet 2 - p.12] The sensor output, FIFO output, and DMP sampling are all
	 * based on the Sample Rate ('Fs' in the datasheet).
	 * 
	 * The Sample Rate is generated by dividing the gyroscope output rate by
	 * SMPLRT_DIV: Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) where
	 * Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or 7),
	 * and 1kHz when the DLPF is enabled (see Register 26)
	 * 
	 * Note: The accelerometer output rate is 1kHz (accelerometer and not gyroscope
	 * !). This means that for a Sample Rate greater than 1kHz, the same
	 * accelerometer sample may be output to the FIFO, DMP, and sensor registers
	 * more than once.
	 * 
	 * @return the sample rate, in Hz.
	 */
	public int getSampleRate() {
		int gyroscopeOutputRate = dlpfCfg == 0 || dlpfCfg == 7 ? 8000 : 1000; // 8kHz if DLPG disabled, and 1kHz if
																				// enabled.
		return gyroscopeOutputRate / (1 + smplrtDiv);
	}

	/**
	 * Sets the value of the DLPF config, according to the datasheet informations.
	 * 
	 * The accelerometer and gyroscope are filtered according to the value of
	 * DLPF_CFG as shown in the table [datasheet 2 - p.13].
	 * 
	 * @param dlpfConfig the new DLPF_CFG value. Must be in the [0; 7] range, where
	 *                   0 and 7 are used to disable the DLPF.
	 */
	public void setDLPFConfig(int dlpfConfig) {
		if (dlpfConfig > 7 || dlpfConfig < 0)
			throw new IllegalArgumentException("The DLPF config must be in the 0..7 range.");
		dlpfCfg = dlpfConfig;
		updateRegisterValue(MPU9250_REG_ADDR_CONFIG, dlpfCfg);
	}

	/**
	 * Reads the most recent accelerometer values on MPU6050 for X, Y and Z axis,
	 * and calculates the corresponding accelerations in g, according to the
	 * selected AFS_SEL mode.
	 * 
	 * @return [ACCEL_X, ACCEL_Y, ACCEL_Z], the accelerations in g for the x, y and
	 *         z axis.
	 */
	public double[] readScaledAccelerometerValues() {
		double accelX = readWord2C(MPU9250_REG_ADDR_ACCEL_XOUT_H);
		accelX /= accelLSBSensitivity;
		double accelY = readWord2C(MPU9250_REG_ADDR_ACCEL_YOUT_H);
		accelY /= accelLSBSensitivity;
		double accelZ = readWord2C(MPU9250_REG_ADDR_ACCEL_ZOUT_H);
		accelZ /= accelLSBSensitivity;

		return new double[] { accelX, accelY, -accelZ };
	}

	/**
	 * Reads the most recent gyroscope values on the MPU6050 for X, Y and Z axis,
	 * and calculates the corresponding angular speeds in degrees/sec, according to
	 * the selected FS_SEL mode.
	 * 
	 * @return [GYRO_X, GYRO_Y, GYRO_Z], the angular velocities in degrees/sec for
	 *         the x, y and z axis.
	 */
	public double[] readScaledGyroscopeValues() {
		double gyroX = readWord2C(MPU9250_REG_ADDR_GYRO_XOUT_H);
		gyroX /= gyroLSBSensitivity;
		double gyroY = readWord2C(MPU9250_REG_ADDR_GYRO_YOUT_H);
		gyroY /= gyroLSBSensitivity;
		double gyroZ = readWord2C(MPU9250_REG_ADDR_GYRO_ZOUT_H);
		gyroZ /= gyroLSBSensitivity;

		return new double[] { gyroX, gyroY, gyroZ };
	}

	/**
	 * Calibrate the accelerometer and gyroscope sensors.
	 */
	private void calibrateSensors() {
		// Robot.robotLogger.log(Logger.INFO, this, "Calibration starting in 3 seconds
		// (don't move the sensor)");
		System.out.println("Gyro Calibration Beginning...");
		pause(3000); // let the MPU stabilize for 3 seconds

		// we really should not be doing the logo here but it makes it easy and
		// we are running out of time
		// Robot.robotScreen.updateBitmap(OLEDBitmap.GAME.getData(),
		// OLEDBitmap.LOGO.getWidth(),
		// OLEDBitmap.LOGO.getHeight(), 0, 0);

		// Robot.robotLogger.log(Logger.INFO, this, "Calibration will take aprox 5
		// seconds (don't move the sensor)");

		// during calibration we take 50 readings, 100ms apart
		// one all readings have been collected, they are averaged
		// if calibration is taking longer than desired, the number of
		// readings could be reduced with a corresponding margin of error
		int nbReadings = 50;

		// Gyroscope offsets
		gyroAngularSpeedOffsetX = 0.;
		gyroAngularSpeedOffsetY = 0.;
		gyroAngularSpeedOffsetZ = 0.;
		for (int i = 0; i < nbReadings; i++) {
			double[] angularSpeeds = readScaledGyroscopeValues();
			gyroAngularSpeedOffsetX += angularSpeeds[0];
			gyroAngularSpeedOffsetY += angularSpeeds[1];
			gyroAngularSpeedOffsetZ += angularSpeeds[2];
			// Robot.robotScreen.updateGraphBar(0, Robot.robotScreen.getHeight() - 8
			// ,Robot.robotScreen.getWidth(), 8,
			// ((double) (i + 1) / (double) nbReadings));
			pause(100);
		}
		gyroAngularSpeedOffsetX /= nbReadings;
		gyroAngularSpeedOffsetY /= nbReadings;
		gyroAngularSpeedOffsetZ /= nbReadings;

		// we really should not be doing the logo here but it makes it easy and
		// we are running out of time
		// Robot.robotScreen.updateBitmap(OLEDBitmap.READY.getData(),
		// OLEDBitmap.LOGO.getWidth(),
		// OLEDBitmap.LOGO.getHeight(), 0, 0);

		// Robot.robotLogger.log(Logger.INFO, this, "Calibration ended");
		System.out.println("Gyro Calibration Finished!");
	}

	/**
	 * Starts the thread responsible to update MPU6050 values in background.
	 */
	public void startUpdatingThread() {
		if (updatingThread == null || !updatingThread.isAlive()) {
			updatingThreadStopped = false;
			lastUpdateTime = System.currentTimeMillis();
			updatingThread = new Thread(() -> {
				while (!updatingThreadStopped) {
					updateValues();
				}
			});
			updatingThread.start();
		} else {
			// Robot.robotLogger.log(Logger.INFO, this, "Updating thread of the MPU9250 is
			// already started.");
			System.out.println("MPU thread already started!");
		}
	}

	/**
	 * Stops the thread responsible to update MPU6050 values in background.
	 * 
	 * @throws InterruptedException if any thread has interrupted the current
	 *                              thread. The interrupted status of the current
	 *                              thread is cleared when this exception is thrown.
	 */
	public void stopUpdatingThread() throws InterruptedException {
		updatingThreadStopped = true;
		try {
			updatingThread.join();
		} catch (InterruptedException e) {
			// Robot.robotLogger.log(Logger.ERROR, this, "Exception when joining the
			// updating thread.");
			throw e;
		}
		updatingThread = null;
	}

	/**
	 * Reset values for the accelerometer angles, gyroscope angles and filtered
	 * angles values back to 0.0.
	 * 
	 * caution should be used that this method is not called when multiple objects
	 * are fetching gyro/accel data values concurrently
	 */
	public void resetValues() {
		// Accelerometer
		accelAccelerationX = 0.0;
		accelAccelerationY = 0.0;
		accelAccelerationZ = 0.0;
		accelAngleX = 0.0;
		accelAngleY = 0.0;
		accelAngleZ = 0.0;

		// Gyroscope
		gyroAngularSpeedX = 0.0;
		gyroAngularSpeedY = 0.0;
		gyroAngularSpeedZ = 0.0;
		// angular speed * time = angle
		gyroAngleX = 0.0;
		gyroAngleY = 0.0;
		gyroAngleZ = 0.0;

		// Low Pass Filtered vales
		filteredAngleX = 0.0;
		filteredAngleY = 0.0;
		filteredAngleZ = 0.0;
	}

	/**
	 * Update values for the accelerometer angles, gyroscope angles and filtered
	 * angles values.
	 * <p>
	 * <i>This method is used with the updating thread.</i>
	 * </p>
	 */
	private void updateValues() {
		// Accelerometer
		double[] accelerations = readScaledAccelerometerValues();
		accelAccelerationX = accelerations[0];
		accelAccelerationY = accelerations[1];
		accelAccelerationZ = accelerations[2];
		accelAngleX = getAccelXAngle(accelAccelerationX, accelAccelerationY, accelAccelerationZ);
		accelAngleY = getAccelYAngle(accelAccelerationX, accelAccelerationY, accelAccelerationZ);
		accelAngleZ = getAccelZAngle();

		// Gyroscope
		double[] angularSpeeds = readScaledGyroscopeValues();
		gyroAngularSpeedX = angularSpeeds[0] - gyroAngularSpeedOffsetX;
		gyroAngularSpeedY = angularSpeeds[1] - gyroAngularSpeedOffsetY;
		gyroAngularSpeedZ = angularSpeeds[2] - gyroAngularSpeedOffsetZ;
		// angular speed * time = angle
		double dt = Math.abs(System.currentTimeMillis() - lastUpdateTime) / 1000.; // s
		double deltaGyroAngleX = gyroAngularSpeedX * dt;
		double deltaGyroAngleY = gyroAngularSpeedY * dt;
		double deltaGyroAngleZ = gyroAngularSpeedZ * dt;
		lastUpdateTime = System.currentTimeMillis();

		gyroAngleX += deltaGyroAngleX;
		gyroAngleY += deltaGyroAngleY;
		gyroAngleZ += deltaGyroAngleZ;

		// Low Pass Filter
		// decreasing alpha will increase the filtering
		// @see
		// https://github.com/KalebKE/AccelerationExplorer/wiki/Signal-Noise-and-Noise-Filters
		// tests should be performed once the MPU has been installed on the robot to
		// determine the best value
		double alpha = 0.96;
		filteredAngleX = alpha * (filteredAngleX + deltaGyroAngleX) + (1. - alpha) * accelAngleX;
		filteredAngleY = alpha * (filteredAngleY + deltaGyroAngleY) + (1. - alpha) * accelAngleY;
		filteredAngleZ = filteredAngleZ + deltaGyroAngleZ;
	}

	/**
	 * Get the last acceleration values, in deg, retrieved from the accelerometer,
	 * for the x, y and z axis.
	 * <p>
	 * <i>(using the updating thread)</i>
	 * </p>
	 * 
	 * @return the accelerations for the x, y and z axis. [-1, -1, -1] if the
	 *         updating thread isn't running.
	 */
	public double[] getAccelAccelerations() {
		if (updatingThreadStopped)
			return new double[] { -1., -1., -1. };
		return new double[] { accelAccelerationX, accelAccelerationY, accelAccelerationZ };
	}

	/**
	 * Get the last angle values, in deg, retrieved from the accelerometer, for the
	 * x, y and z axis.
	 * <p>
	 * <i>(using the updating thread)</i>
	 * </p>
	 * 
	 * @return the angle values for the x, y and z axis. [-1, -1, -1] if the
	 *         updating thread isn't running.
	 */
	public double[] getAccelAngles() {
		if (updatingThreadStopped)
			return new double[] { -1., -1., -1. };
		return new double[] { accelAngleX, accelAngleY, accelAngleZ };
	}

	/**
	 * Get the last angular speed values, in deg/sec, retrieved from the gyroscope,
	 * for the x, y and z axis.
	 * <p>
	 * <i>(using the updating thread)</i>
	 * </p>
	 * 
	 * @return the angular speed values for the x, y and z axis. [-1, -1, -1] if the
	 *         updating thread isn't running.
	 */
	public double[] getGyroAngularSpeeds() {
		if (updatingThreadStopped)
			return new double[] { -1., -1., -1. };
		return new double[] { gyroAngularSpeedX, gyroAngularSpeedY, gyroAngularSpeedZ };
	}

	/**
	 * Get the last angles values, in deg, retrieved from the gyroscope, for the x,
	 * y and z axis.
	 * <p>
	 * <i>(using the updating thread)</i>
	 * </p>
	 * 
	 * @return the angles values from the gyroscope for the x, y and z axis. [-1,
	 *         -1, -1] if the updating thread isn't running.
	 */
	public double[] getGyroAngles() {
		if (updatingThreadStopped)
			return new double[] { -1., -1., -1. };
		return new double[] { gyroAngleX, gyroAngleY, gyroAngleZ };
	}

	/**
	 * Get the calculated offsets for the angular speeds from the gyroscope, for the
	 * x, y and z axis.
	 * <p>
	 * <i>(calculated with the calibration function)</i>
	 * </p>
	 * 
	 * @return the offsets for the angular speeds from the gyroscope.
	 */
	public double[] getGyroAngularSpeedsOffsets() {
		return new double[] { gyroAngularSpeedOffsetX, gyroAngularSpeedOffsetY, gyroAngularSpeedOffsetZ };
	}

	/**
	 * Last angle value, in deg, calculated from the accelerometer and the
	 * gyroscope, for the x, y and z axis.
	 * <p>
	 * <i>(using the updating thread)</i>
	 * </p>
	 * 
	 * @return the angles values, in deg, filtered with values from the
	 *         accelerometer and the gyroscope.
	 */
	public double[] getFilteredAngles() {
		if (updatingThreadStopped)
			return new double[] { -1., -1., -1. };
		return new double[] { filteredAngleX, filteredAngleY, filteredAngleZ };
	}

	/**
	 * Last angle value, in deg, calculated from the accelerometer and the
	 * gyroscope, for the x axis aka Roll.
	 * <p>
	 * <i>(using the updating thread)</i>
	 * </p>
	 * 
	 * @return the angle value, in deg, filtered with values from the accelerometer
	 *         and the gyroscope.
	 */
	public double getFilteredRoll() {
		return filteredAngleX;
	}

	/**
	 * Last angle value, in deg, calculated from the accelerometer and the
	 * gyroscope, for the y axis aka Roll.
	 * <p>
	 * <i>(using the updating thread)</i>
	 * </p>
	 * 
	 * @return the angle value, in deg, filtered with values from the accelerometer
	 *         and the gyroscope.
	 */
	public double getFilteredPitch() {
		return filteredAngleY;
	}

	/**
	 * Last angle value, in deg, calculated from the accelerometer and the
	 * gyroscope, for the z axis aka Roll.
	 * <p>
	 * <i>(using the updating thread)</i>
	 * </p>
	 * 
	 * @return the angle value, in deg, filtered with values from the accelerometer
	 *         and the gyroscope.
	 */
	public double getFilteredYaw() {
		return filteredAngleZ;
	}

	/*
	 * ----------------------------------------------------------------------- UTILS
	 * -----------------------------------------------------------------------
	 */

	/**
	 * This method updates the value of a specific register with a specific value.
	 * The method also checks that the update was successful.
	 * 
	 * @param address the address of the register to update.
	 * @param value   the new value to set in the register.
	 */
	public void updateRegisterValue(int address, int value) {
		this.writeByte(address, value);
		// we check that the value of the register has been updated
		// byte[] data = new byte[1];
		// boolean failed = readByte(address, data);
		// if (failed)
		// Robot.robotLogger.log(Logger.ERROR, this, "confirmation read failed");
		// if (data[0] != value)
		// Robot.robotLogger.log(Logger.WARNING, this, "Error when updating the MPU9250
		// register value(register: "
		// + Integer.toHexString(address) + ",value:" + value + ")");
	}

	/**
	 * Reads the content of a specific register of the MPU6050.
	 * 
	 * @param registerAddress the address of the register to read.
	 * @return the int representation of the content of the register.
	 */
	// private int readRegisterValue(int registerAddress) {
	// byte[] data = new byte[1];
	// boolean failed = readByte(registerAddress, data);
	// if (failed)
	// Robot.robotLogger.log(Logger.INFO, this, "read failed");
	// return (int) data[0];
	// }

	/**
	 * Reads the content of two consecutive registers, starting at registerAddress,
	 * and returns the int representation of the combination of those registers,
	 * with a two's complement representation.
	 * 
	 * @param registerAddress the address of the first register to read.
	 * @return the int representation of the combination of the two consecutive
	 *         registers, with a two's complement representation.
	 */
	private int readWord2C(int registerAddress) {
		short[] data = new short[1];
		// boolean failed = readShort(registerAddress, data);
		// if (failed)
		// Robot.robotLogger.log(Logger.INFO, this, "read failed");
		return (int) data[0];
	}

	/**
	 * Get the distance between two points.
	 * 
	 * @param a the first point.
	 * @param b the second point.
	 * @return the distance between a and b.
	 */
	private double distance(double a, double b) {
		return Math.sqrt(a * a + b * b);
	}

	private double getAccelXAngle(double x, double y, double z) {
		// v1 - 360
		double radians = Math.atan2(y, distance(x, z));
		double delta = 0.;
		if (y >= 0) {
			if (z >= 0) {
				// pass
			} else {
				radians *= -1;
				delta = 180.;
			}
		} else {
			if (z <= 0) {
				radians *= -1;
				delta = 180.;
			} else {
				delta = 360.;
			}
		}
		return radians * RADIAN_TO_DEGREE + delta;
	}

	private double getAccelYAngle(double x, double y, double z) {
		// v2
		double tan = -1 * x / distance(y, z);
		double delta = 0.;
		if (x <= 0) {
			if (z >= 0) {
				// q1
				// nothing to do
			} else {
				// q2
				tan *= -1;
				delta = 180.;
			}
		} else {
			if (z <= 0) {
				// q3
				tan *= -1;
				delta = 180.;
			} else {
				// q4
				delta = 360.;
			}
		}

		return Math.atan(tan) * RADIAN_TO_DEGREE + delta;
	}

	private double getAccelZAngle() {
		return ACCEL_Z_ANGLE;
	}

	/**
	 * Returns the String representation of an angle, in the "x.xxxx deg" format.
	 * 
	 * @param angle the angle to convert.
	 * @return the String representation of an angle, in the "x.xxxx deg" format.
	 */
	public static String angleToString(double angle) {
		return String.format("%.4f", angle) + "deg";
	}

	/**
	 * Returns the String representation of an acceleration value, in the
	 * "x.xxxxxxg" format.
	 * 
	 * @param accel the acceleration to convert.
	 * @return the String representation of an acceleration value, in the
	 *         "x.xxxxxxg" format.
	 */
	public static String accelToString(double accel) {
		return String.format("%.6f", accel) + "g";
	}

	/**
	 * Returns the String representation of an angular speed value, in the "x.xxxx
	 * deg/s" format.
	 * 
	 * @param angularSpeed the angular speed to convert.
	 * @return the String representation of an angular speed value, in the "x.xxxx
	 *         deg/s" format.
	 */
	public static String angularSpeedToString(double angularSpeed) {
		return String.format("%.4f", angularSpeed) + "deg/s";
	}

	/**
	 * Returns a String representation of a triplet of values, in the "x: X\t y: Y\t
	 * z: Z" format.
	 * 
	 * @param x the first value of the triplet.
	 * @param y the second value of the triplet.
	 * @param z the thirs value of the triplet.
	 * @return a String representation of a triplet of values, in the "x: X\t y: Y\t
	 *         z: Z" format.
	 */
	public static String xyzValuesToString(String x, String y, String z) {
		return "x: " + x + "\ty: " + y + "\tz: " + z;
	}

	private void pause(long duration) {
		try {
			Thread.sleep(duration);
		} catch (InterruptedException e) {
			// Robot.robotLogger.log(Logger.WARNING, this, "ignoring interrupt exception");
		}
	}
}