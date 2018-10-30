package lab3;

import java.util.Arrays;

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

/*Collaborators:
 * Tricia Bacon (730011125)
 * Sarah Ganci (720510446)
 * 
 * */

public class Charlieold {
	// components
	private EV3LargeRegulatedMotor motorL;
	private EV3LargeRegulatedMotor motorR;
	private EV3MediumRegulatedMotor motorM;
	private EV3TouchSensor touchSensorL;
	private EV3TouchSensor touchSensorR;
	private EV3UltrasonicSensor sonicSensor;
	private EV3GyroSensor gyroSensor;
	private double radiusL;
	private double radiusR;
	private double L;
	private float heading;

	// sensor modes
	private SensorMode touchL;
	private SensorMode touchR;
	private SensorMode sonic;
	private SensorMode gyro;

	public Charlieold() {
		this.motorL = new EV3LargeRegulatedMotor(MotorPort.B);
		this.motorR = new EV3LargeRegulatedMotor(MotorPort.C);
		this.motorM = new EV3MediumRegulatedMotor(MotorPort.A);
		this.touchSensorL = new EV3TouchSensor(SensorPort.S1);
		this.touchSensorR = new EV3TouchSensor(SensorPort.S4);
		this.sonicSensor = new EV3UltrasonicSensor(SensorPort.S2);
		this.gyroSensor = new EV3GyroSensor(SensorPort.S3);

		this.touchL = this.touchSensorL.getTouchMode();
		this.touchR = this.touchSensorR.getTouchMode();
		this.sonic = (SensorMode) this.sonicSensor.getDistanceMode();
		this.gyro = (SensorMode) this.gyroSensor.getAngleMode();
		this.radiusL = .028;
		this.radiusR = .028;
		this.L = .12;
		this.heading = 0;
	}

	/*
	 * Name: setLeftSpeed in: angular velocity in degrees (float s) out: nothing
	 * description: sets left motor's speed to that angular velocity
	 */
	public void setLeftSpeed(float s) {
		this.motorL.setSpeed(s);
	}

	/*
	 * Name: setRighttSpeed in: angular velocity in degrees (float s) out: nothing
	 * description: sets right motor's speed to that angular velocity
	 */
	public void setRightSpeed(float s) {
		this.motorR.setSpeed(s);
	}

	/*
	 * Name: setBothSpeed in: angular velocity in degrees (float s) out: nothing
	 * description: sets both motors speed to s
	 */
	public void setBothSpeed(float s) {
		this.setLeftSpeed(s);
		this.setRightSpeed(s);
	}

	/*
	 * Name: moveForwardBoth in: nothing out: nothing description: moves both motors
	 * forward
	 */
	public void moveForwardBoth() {
		this.motorL.forward();
		this.motorR.forward();
	}

	/*
	 * Name: moveBackwardBoth in: nothing out: nothing description: moves both
	 * motors backward
	 */
	public void moveBackwardBoth() {
		this.motorL.backward();
		this.motorR.backward();
	}

	/*
	 * Name: stopBothInstant in: nothing out: nothing description: stops both
	 * instantly
	 */
	public void stopBothInstant() {
		this.motorL.stop(true);
		this.motorR.stop(true);
	}

	/*
	 * Name: moveTillSense in: distance in meters (dist) out: distance traveled
	 * description: moves robot forward till sonic sensor senses an obstacle a
	 * certain distance away
	 */
	public void moveTillSense(double d) {
		this.syncMotors();
		float speed = 180;
		this.setBothSpeed(speed);
		float[] sample_sonic = new float[this.sonic.sampleSize()];
		this.sonic.fetchSample(sample_sonic, 0);
		long startTime = System.nanoTime();
		while (sample_sonic[0] > d) {
			this.moveForwardBoth();
			sonic.fetchSample(sample_sonic, 0);
		}
		this.stopBothInstant();
		long time = (System.nanoTime() - startTime);
		// this.getPositionStraight(this.heading, speed, time);
		this.stopSync();
	}

	/*
	 * Name: moveTillTouch in: nothing out: nothing description: moves robot forward
	 * till touch sensor encounters an obstacle
	 */
	public void moveTillTouch() {
		this.syncMotors();
		float[] sample_touchL = new float[touchL.sampleSize()];
		float[] sample_touchR = new float[touchR.sampleSize()];

		while (sample_touchL[0] == 0 && sample_touchR[0] == 0) {
			this.moveForwardBoth();
			touchL.fetchSample(sample_touchL, 0);
			touchR.fetchSample(sample_touchR, 0);
		}
		this.stopBothInstant();
		this.stopSync();
	}

	/*
	 * Name: syncMotors in: nothing out: nothing description: synchronizes left and
	 * right motors
	 */
	public void syncMotors() {
		this.motorL.synchronizeWith(new EV3LargeRegulatedMotor[] { this.motorR });
	}

	public void stopSync() {
		this.motorL.endSynchronization();
	}

	/*
	 * Name: moveForwardTime in: time to move in seconds out: nothing description:
	 * makes robot move forward for a certain amount of time
	 */
	public void moveForwardTime(long sec) {
		this.syncMotors();
		this.moveForwardBoth();
		Delay.msDelay(sec);
		this.stopBothInstant();
		this.stopSync();
	}

	/*
	 * Name: moveBackwardTime in: time to move in seconds out: nothing description:
	 * makes robot move Backward for a certain amount of time
	 */
	public void moveBackwardTime(long sec) {
		this.syncMotors();
		this.moveBackwardBoth();
		Delay.msDelay(sec);
		this.stopBothInstant();
		this.stopSync();
	}

	/////////////////////////////////// need to alter move time to account for diff
	/////////////////////////////////// radius
	/*
	 * Name: moveTime in: time to move in seconds out: nothing description: makes
	 * robot move Backward for a certain amount of time
	 */
	private long moveTime(double vr, double vl, double d) {
		// d = v*t => t = d/v
		// v = (vr+vl)/2
		return (long) (d / ((vr + vl) / 2) * 1000);
	}

	/*
	 * Name: moveForwardDist in: distance in meters out: nothing description: moves
	 * robot forward a certain distance
	 **/
	public void moveForwardDist(double d) {
		float speed = 180;
		this.setBothSpeed(speed);
		double vl = speed * (Math.PI / 180) * this.radiusL;
		double vr = speed * (Math.PI / 180) * this.radiusR;
		long sec = this.moveTime(vr, vl, d);
		this.moveForwardTime(sec);
	}

	/*
	 * Name: moveBackwardDist in: distance in meters out: nothing description: moves
	 * robot backward a certain distance
	 **/
	public void moveBackwardDist(double d) {
		float speed = 180;
		this.setBothSpeed(speed);
		double vl = speed * (Math.PI / 180) * this.radiusL;
		double vr = speed * (Math.PI / 180) * this.radiusR;
		long sec = this.moveTime(vr, vl, d);
		this.moveBackwardTime(sec);
	}

	/*
	 * Name: beep in: nothing out: nothing description: makes robot beep
	 */
	public void beep() {
		Sound.beep();
	}

	/*
	 * Name: buttonWait in: nothing out: nothing description: waits for button press
	 * to continue
	 */
	public void buttonWait() {
		Button.ENTER.waitForPressAndRelease();
	}

	///////////// NEW

	public void trace() {
		// 1: take a valid reading from sonic
		float sonic = this.sonicSense();

		// 2: set initial speeds and move forward
		float bs = 270;
		this.setBothSpeed(bs);
		this.syncMotors();
		this.moveForwardBoth();

		double l1 = 0.08 / Math.cos(5 * Math.PI / 36);
		double l2 = 0.12 / Math.cos(5 * Math.PI / 36);
		double l3 = 0.18 / Math.cos(5 * Math.PI / 36);
		double l4 = 0.22 / Math.cos(5 * Math.PI / 36);
		double l5 = 0.3 / Math.cos(5 * Math.PI / 36);

		while (sonic < 0.5) {

			if (sonic <= .05) {
				// Option 1: way too close -- move to the right fast
				this.setDiffSpeeds(270, 180);
			} else if (sonic < .12) {
				// Option 2: way too close -- move to the right fast
				this.setDiffSpeeds(240, 180);
			} else if (sonic < .14) {
				// Option 3: a little to close to wall -- adjust right a little
				this.setDiffSpeeds(210, 180);
			} else if (sonic >= .14 && sonic <= .16) {
				// Option 4: just right -- set wheels to same speed and move on forward
				this.setBothSpeed(180);
			} else if (sonic > .16 && sonic <= .2) {
				// Option 5: a little too far from wall -- move left slow
				this.setDiffSpeeds(180, 210);
			} else if (sonic > .2 && sonic <= .25) {
				// Option 6: a little too far from wall -- move left slow
				this.setDiffSpeeds(180, 240);
			} else { // .2
				// Option 7: way too far -- move to the left fast
				this.setDiffSpeeds(180, 270);
			}
			sonic = this.sonicSense();
			if (sonic > .4) {
//				this.motorM.rotateTo((int) this.heading);
//				float sonic1 = this.sonicSense();
//				this.motorM.rotate(45);
//				float sonic2 = this.sonicSense();
				System.out.println("in greater than .4 statement " + sonic);
				this.stopBothInstant();
//				this.rotateLeft(30);
				sonic = sonicSense();
				if (sonic > .5) {
					break;
				}
			}
		}
		System.out.println("End loop");
		this.stopBothInstant();
		this.stopSync();
	}

	public void setProportionalSpeeds(float son, float angle) {
		double norm = son / 0.3;
		// double addeg= (1.0-norm)*90.0;
		if (norm < 0.5) {
			float addeg = (float) (1.0 - norm) * angle;
			this.setLeftSpeed(180 + addeg);

		} else {
			float addeg = (float) (norm) * angle;
			this.setRightSpeed(180 + addeg);
		}
	}

	public void setDiffSpeeds(float left, float right) {
		this.setLeftSpeed(left);
		this.setRightSpeed(right);
	}

	/*
	 * Name: rotateSonic in: degrees out: nothing description: should rotate sonic
	 * sensor to certain angle relative to robot
	 * 
	 */
	public void rotateSonic(int degrees) {
		this.motorM.setSpeed(90);
		this.motorM.rotate(degrees);
	}

	/*
	 * Name: sonicSense in: nothing out: double number representing the median of
	 * the distance sensed from the sonic sensor description: senses 3 times and
	 * returns the median
	 */

	public float sonicSense() {
		float[] dists = new float[3];
		float[] sample_sonic = new float[sonic.sampleSize()];
		for (int i = 0; i < dists.length; i++) {
			// System.out.println("Sensing Sonic");
			sonic.fetchSample(sample_sonic, 0);
			dists[i] = sample_sonic[0];
			int loop = 0;
			while (dists[i] > .8 && loop < 5) {
				// System.out.println("Infinity " + loop);
				sonic.fetchSample(sample_sonic, 0);
				dists[i] = sample_sonic[0];
				loop++;
			}
		}
		Arrays.sort(dists);
		return dists[1];
	}

	/*
	 * Name: moveToward in: x: translation over x axis, y: translation over y axis
	 * stop: boolean (stop once point is reached or keep moving in that direction)
	 * out: none description:
	 */

	public void moveToward(double x, double y) {

	}

	/*
	 * Name: rotateRight in: degrees to rotate out: nothing description: should turn
	 * the robot in place towards goal
	 */

	public void rotateRight(long degrees) {
		// move right forward and left backward to create a spin
		this.stopSync();
		this.motorL.forward();
//		this.motorR.backward();

//		long delay= (long) (degrees/av)*1000; // need to set delay time
		this.setLeftSpeed(180);
		long delay = this.timeToRotate(0, 180, degrees);
		Delay.msDelay(delay);

		// stop both motors
		this.stopBothInstant();
//		this.heading += degrees;

	}

	/*
	 * Name: rotateLeft in: degrees to rotate out: nothing description: should turn
	 * the robot in place towards goal
	 */

	public void rotateLeft(int degrees) {
		System.out.println("RotateLeft");

		// move right forward and left backward to create a spin
		this.stopSync();
		this.motorR.forward();

		this.setRightSpeed(180);
		long delay = this.timeToRotate(180, 0, degrees);
		Delay.msDelay(delay);

		// stop both motors
		this.stopBothInstant();
//		this.heading -= degrees;
	}

	public float theta() {
		float[] sample_gyro = new float[gyro.sampleSize()];
		this.gyro.fetchSample(sample_gyro, 0);
		return sample_gyro[0];
	}

	public void setHeading() {
		this.heading = this.theta();
	}

	public float getHeading() {
		return this.heading;
	}

//	/*Name: timeToRotate
//	 * in: ul, ur, theta
//	 * out: seconds to move
//	 * */
//	
	public long timeToRotate(double ur, double ul, double theta) {
		double vr = ur * Math.PI / 180 * this.radiusR;
		double vl = ul * Math.PI / 180 * this.radiusL;
		double omega = (vl - vr) / this.L;
		double time = (theta * Math.PI / 180) / omega;
		return (long) time * 1000;
	}

///////////////////////////////////////////////// Sensing

	public boolean leftBump() {
		float[] sample_touchL = new float[touchL.sampleSize()];
		touchL.fetchSample(sample_touchL, 0);
		return sample_touchL[0] != 0;
	}

	public boolean frontBump() {
		float[] sample_touchR = new float[touchR.sampleSize()];
		touchR.fetchSample(sample_touchR, 0);
		return sample_touchR[0] != 0;
	}

	public void trace2() {
		// 1: take a valid reading from sonic
		float sonic = this.sonicSense();

		// 2: set initial speeds and move forward
		float bs = 270;
		this.setBothSpeed(bs);
		this.syncMotors();
		this.moveForwardBoth();

		// NOTE: change looping condition to check for orientation
		boolean orient = true;
		// 3: loop till return to origin
		while (!Button.ENTER.isDown()) {

			// 4: check to see if wall is bumped
			if (this.frontBump()) {
				System.out.println("front bump");
				this.stopBothInstant();
				// 4.1: back up body length
				this.moveBackwardDist(0.15);

				// 4.3: turn
				this.rotateRight(105);

				// 4.4: move forward
				this.moveForwardBoth();
				this.setBothSpeed(270);

				// 4.5: continue on to next iteration of big loop
				continue;
			} else if (this.leftBump()) {
				System.out.println("left bump");
				this.setDiffSpeeds(180, 270);
				continue;
			}

			double dbuff = 0;
			double d1 = .14 + dbuff;
			double d2 = .19 + dbuff;
			double d3 = .23 + dbuff;
			double d4 = .28 + dbuff;
			int s1 = 180;
			int s2 = 210;
			int s3 = 240;
			int s4 = 270;

			// 6: Adjust speed based on distance
			// note: sonic is 6 cm from the bump

			if (sonic <= d1) {
				// Option 1: way too close -- move to the right fast
				System.out.println("too close: " + sonic);
				this.setDiffSpeeds(s4, s1);
			} else if (sonic > d1 && sonic <= d2) {
				// Option 2: too close -- move to the right fast
				System.out.println("close: " + sonic);
				this.setDiffSpeeds(s3, s2);
			} else if (sonic > d2 && sonic < d3) {
				System.out.println("perfect: " + sonic);
				// Option 3: perf
				this.setBothSpeed(s4);
			} else if (sonic >= d3 && sonic <= d4) {
				System.out.println("far: " + sonic);
				// Option 4: a little too far from wall
				this.setDiffSpeeds(s2, s3);

			} else {
				System.out.println("too far: " + sonic);
				// option 5: way too far from wall
				this.setDiffSpeeds(s1, s4);
			}

			sonic = this.sonicSense();
		}
		System.out.println("End loop");
		this.stopBothInstant();
		this.stopSync();
	}

	public void trace3() {
		// 1: take a valid reading from sonic
		float sonic = this.sonicSense();

		// 2: set initial speeds and move forward
		float bs = 270;
		this.setBothSpeed(bs);
		this.syncMotors();
		this.moveForwardBoth();

		// NOTE: change looping condition to check for orientation
		boolean orient = true;
		// 3: loop till return to origin
		while (!Button.ENTER.isDown()) {

			// 4: check to see if wall is bumped
			if (this.frontBump()) {
				System.out.println("front bump");
				this.stopBothInstant();
				// 4.1: back up body length
				this.moveBackwardDist(0.15);

				// 4.3: turn
				this.rotateRight(105);

				// 4.4: move forward
				this.moveForwardBoth();
				this.setBothSpeed(270);

				// 4.5: continue on to next iteration of big loop
				continue;
			} else if (this.leftBump()) {
				System.out.println("left bump");
				this.setDiffSpeeds(180, 270);
				continue;
			}

			double dbuff = .06;
			double d1 = 0.05 + dbuff;
			double d2 = 0.12 + dbuff;
			double d3 = 0.14 + dbuff;
			double d4 = .16 + dbuff;
			double d5 = .18 + dbuff;
			double d6 = .25 + dbuff;
			if (sonic <= d1) {
				// Option 1: way too close -- move to the right fast
				this.setDiffSpeeds(270, 180);
			} else if (sonic < d2) {
				// Option 2: way too close -- move to the right fast
				this.setDiffSpeeds(240, 180);
			} else if (sonic < d3) {
				// Option 3: a little to close to wall -- adjust right a little
				this.setDiffSpeeds(210, 180);
			} else if (sonic >= d3 && sonic <= d4) {
				// Option 4: just right -- set wheels to same speed and move on forward
				this.setBothSpeed(180);
			} else if (sonic > d4 && sonic <= d5) {
				// Option 5: a little too far from wall -- move left slow
				this.setDiffSpeeds(180, 210);
			} else if (sonic > d5 && sonic <= d6) {
				// Option 6: a little too far from wall -- move left slow
				this.setDiffSpeeds(180, 240);
			} else { // .2
				// Option 7: way too far -- move to the left fast
				this.setDiffSpeeds(180, 270);
			}
			sonic = this.sonicSense();
		}
		System.out.println("End loop");
		this.stopBothInstant();
		this.stopSync();

	}

	/*
	 * Name: sonicRotateSense in: none out: float array holding 3 sensed distance
	 * values
	 */

	// problems: sonic now sensing all 0s for some reason? why?
	public float[] sonicRotateSense() {
		float[] senses = new float[4];

		// original angle is -90

		// rotate back -45 degrees to sense behind (angle now at -135)
		this.rotateSonic(-45);
		senses[0] = this.sonicSense();
		System.out.println(senses[0]);

		// rotate forward 50 degrees to sense where position was but a little past to
		// account for exact reflection
		// angle now at -85
		this.rotateSonic(50);
		senses[1] = this.sonicSense();
		System.out.println(senses[1]);

		// rotate forward 40 (now angle is -45) so its looking ahead
		this.rotateSonic(40);
		senses[2] = this.sonicSense();
		System.out.println(senses[2]);

		// rotate forward (angel now at 0) and sense
		this.rotateSonic(45);
		senses[3] = this.sonicSense();
		System.out.println(senses[3]);

		// returns sonic to original state
		// angle now at -90
		this.rotateSonic(-90);

		return senses;
	}

	public void rotateLeftTillSense(double dist) {
		this.stopSync();
		this.motorR.forward();
		float sample_sonic = this.sonicSense();
		this.setRightSpeed(180);
		while (sample_sonic > dist) {
			this.motorR.forward();
			sample_sonic = this.sonicSense();
		}
		this.stopBothInstant();
	}

}
