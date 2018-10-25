package lab3;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.utility.Delay;

public class Charles {

	/////////////////////////////////////////////////////////////////////
	/////////////// Section 0: Properties and Constructor ///////////////
	/////////////////////////////////////////////////////////////////////

	// components
	private EV3LargeRegulatedMotor motorL; // left motor attached to left wheel
	private EV3LargeRegulatedMotor motorR; // right motor attached to right wheel
	private EV3MediumRegulatedMotor motorM; // medium motor attached to ultrasonic
	private EV3TouchSensor touchSensorL; // left touch sensor
	private EV3TouchSensor touchSensorF; // front touch sensor
	private EV3UltrasonicSensor sonicSensor; // ultrasonic sensor
	private EV3GyroSensor gyroSensor; // gyro sensor
	private double radiusL; // radius of left wheel
	private double radiusR; // radius of right wheel
	private double L; // distance between the two wheels
	private float originHeading; // orientation

	// sensor modes
	private SensorMode touchL; // left touch sensor mode
	private SensorMode touchF; // front touch sensor mode
	private SensorMode sonic; // sonic distance sensor mode
	private SensorMode gyro; // gyroscopic sensor mode

	public Charles() {
		this.motorL = new EV3LargeRegulatedMotor(MotorPort.B);
		this.motorR = new EV3LargeRegulatedMotor(MotorPort.C);
		this.motorM = new EV3MediumRegulatedMotor(MotorPort.A);
		this.touchSensorL = new EV3TouchSensor(SensorPort.S1);
		this.touchSensorF = new EV3TouchSensor(SensorPort.S4);
		this.sonicSensor = new EV3UltrasonicSensor(SensorPort.S2);
		this.gyroSensor = new EV3GyroSensor(SensorPort.S3);

		this.touchL = this.touchSensorL.getTouchMode();
		this.touchF = this.touchSensorF.getTouchMode();
		this.sonic = (SensorMode) this.sonicSensor.getDistanceMode();
		this.gyro = (SensorMode) this.gyroSensor.getAngleMode();
		this.radiusL = .028;
		this.radiusR = .028;
		this.L = .12;
		this.heading = 0.0;
	}

	////////////////////////////////////////////////////////////
	/////////////// Section 1: Sensing Functions ///////////////
	////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////
	/////////////// Section 2: Motion Functions ///////////////
	///////////////////////////////////////////////////////////

	///// 2.1: Speeds/////
	public void setLeftSpeed(float s) {
		this.motorL.setSpeed(s);
	}

	public void setRightSpeed(float s) {
		this.motorR.setSpeed(s);
	}

	public void setBothSpeed(float s) {
		this.setLeftSpeed(s);
		this.setRightSpeed(s);
	}

	public void setDiffSpeeds(float left, float right) {
		this.setLeftSpeed(left);
		this.setRightSpeed(right);

	}

	///// 2.2: Motors/////
	public void syncMotors() {
		this.motorL.synchronizeWith(new EV3LargeRegulatedMotor[] { this.motorR });
	}

	public void stopSync() {
		this.motorL.endSynchronization();
	}

	///// 2.3: Moving Methods /////
	public void moveForwardBoth() {
		this.motorL.forward();
		this.motorR.forward();
	}

	public void moveBackwardBoth() {
		this.motorL.backward();
		this.motorR.backward();
	}

	public void stopBothInstant() {
		this.motorL.stop(true);
		this.motorR.stop(true);
	}

	private long moveTime(double vr, double vl, double d) {
		// d = v*t => t = d/v
		// v = (vr+vl)/2
		return (long) (d / ((vr + vl) / 2) * 1000);
	}

	public void moveForwardTime(long sec) {
		this.syncMotors();
		this.moveForwardBoth();
		Delay.msDelay(sec);
		this.stopBothInstant();
		this.stopSync();
	}

	public void moveBackwardTime(long sec) {
		this.syncMotors();
		this.moveBackwardBoth();
		Delay.msDelay(sec);
		this.stopBothInstant();
		this.stopSync();
	}

	public void moveForwardDist(double d) {
		float speed = 180;
		this.setBothSpeed(speed);
		double vl = speed * (Math.PI / 180) * this.radiusL;
		double vr = speed * (Math.PI / 180) * this.radiusR;
		long sec = this.moveTime(vr, vl, d);
		this.moveForwardTime(sec);
	}

	public void moveBackwardDist(double d) {
		double vl = this.motorL.getSpeed() * (Math.PI / 180) * this.radiusL;
		double vr = this.motorR.getSpeed() * (Math.PI / 180) * this.radiusR;
		long sec = this.moveTime(vr, vl, d);
		this.moveBackwardTime(sec);
	}

	public void moveTillSense(double d) {
		this.syncMotors();
		float[] sample_sonic = new float[this.sonic.sampleSize()];
		this.sonic.fetchSample(sample_sonic, 0);
		while (sample_sonic[0] > d) {
			this.moveForwardBoth();
			sonic.fetchSample(sample_sonic, 0);
		}
		this.stopBothInstant();
		this.stopSync();
	}

	/*
	 * moveTillTouch (last altered by Sarah)
	 */
	public void moveTillTouch() {
		this.syncMotors();
		float[] sample_touchF = new float[touchF.sampleSize()];

		while (sample_touchF[0] == 0) {
			this.moveForwardBoth();
			touchF.fetchSample(sample_touchF, 0);

			// Robustness Check
			if (sample_touchF[0] != 0) {
				touchF.fetchSample(sample_touchF, 0);
			}
		}
		this.stopBothInstant();
		this.stopSync();
	}

	///// 2.5: Sonic Sensor and Medium Motor Motion /////
	public void rotateSonic(int degrees) {
		this.motorM.setSpeed(90);
		this.motorM.rotate(degrees);
	}

	////////////////////////////////////////////////////////////////
	/////////////// Section 3: Orientation Functions ///////////////
	////////////////////////////////////////////////////////////////

	public float theta() {
		float[] sample_gyro = new float[gyro.sampleSize()];
		this.gyro.fetchSample(sample_gyro, 0);
		return sample_gyro[0];
	}

	public void setOriginHeading() {
		this.originHeading = this.theta();
	}

	public float getOriginHeading() {
		return (float) this.originHeading;
	}

	//////////////////////// NEW/////////////////////////////////

	/*
	 * Name: setX Description: continuously calculates x value (in relation to
	 * origin point (0,0) stores new value
	 */
	public void setX() {

	}

	/*
	 * Name: setY Description: continuously calculates y value (in relation to
	 * origin point (0,0) stores new value
	 */
	public void setY() {

	}

	/////////////////////////////////////////////////////////////////
	/////////////// Section 4: Lab Specific Functions ///////////////
	/////////////////////////////////////////////////////////////////

	public void beep() {
		Sound.beep();
	}

	public void buttonWait() {
		Button.ENTER.waitForPressAndRelease();
	}

	public void trace() {
		// 1: Sense for wall
		// 2: if error, robust check
		while (true) { // loop based on sonic sensor

			// 3: adjust angle based on sonic sense

			/*
			 * 4: read from sonic sensor 4.1: check bump sensor left if left hit move away
			 * from wall read angular sonic sense adjust angle and move towards 4:2: check
			 * bump sensor front if front hit move back from wall read angular sonic sense
			 * adjust based on that 4.3: read from sonic sensor if infinity: rotate sonic
			 * back away from wall and rotate accordingly
			 */

		}

	}

}
