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
 * */

public class Charlie {
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
	private double heading;
	private Coordinate currentPosition;

	// sensor modes
	private SensorMode touchL;
	private SensorMode touchR;
	private SensorMode sonic;
	private SensorMode gyro;

	public Charlie() {
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
		this.heading = 90;
		this.currentPosition = new Coordinate(0, 0);
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
	 * Name: moveTillSense in: distance in meters (dist) out: nothing description:
	 * moves robot forward till sonic sensor senses an obstacle a certain distance
	 * away
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
		this.getPositionStraight(this.heading, speed, time);
		this.stopSync();
	}

	/*
	 * Name: moveTillTouch in: nothing out: nothing description: moves robot forward
	 * till touch sensor encounters an obstacle
	 */
	public void moveTillTouch() {
		this.syncMotors();
		float speed = 180;
		this.setBothSpeed(speed);
		float[] sample_touchL = new float[touchL.sampleSize()];
		float[] sample_touchR = new float[touchR.sampleSize()];
		long startTime = System.nanoTime();
		while (sample_touchL[0] == 0 && sample_touchR[0] == 0) {
			this.moveForwardBoth();
			touchL.fetchSample(sample_touchL, 0);
			touchR.fetchSample(sample_touchR, 0);
		}
		this.stopBothInstant();
		long time = (System.nanoTime() - startTime);
		this.getPositionStraight(this.heading, speed, time);
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
		float speed = 180;
		this.setBothSpeed(speed);
		this.moveForwardBoth();
		Delay.msDelay(sec);
		this.stopBothInstant();
		this.getPositionStraight(this.heading, speed, (long) (sec * Math.pow(10, 9)));
		this.stopSync();
	}

	/*
	 * Name: moveBackwardTime in: time to move in seconds out: nothing description:
	 * makes robot move Backward for a certain amount of time
	 */
	public void moveBackwardTime(long sec) {
		this.syncMotors();
		float speed = 180;
		this.setBothSpeed(speed);
		this.moveBackwardBoth();
		Delay.msDelay(sec);
		this.stopBothInstant();
		this.getPositionStraight(this.heading, speed, (long) (sec * Math.pow(10, 9)));
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
		this.getPositionStraight(this.heading, speed, (long) (sec * Math.pow(10, 9)));
	}

	/*
	 * Name: moveBackwardDist in: distance in meters out: nothing description: moves
	 * robot backward a certain distance
	 **/
	public void moveBackwardDist(double d) {
		float speed = 180;
		this.setBothSpeed(speed);
		double vl = this.motorL.getSpeed() * (Math.PI / 180) * this.radiusL;
		double vr = this.motorR.getSpeed() * (Math.PI / 180) * this.radiusR;
		long sec = this.moveTime(vr, vl, d);
		this.moveBackwardTime(sec);
		this.getPositionStraight(this.heading, speed, (long) (sec * Math.pow(10, 9)));
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

		long startTime = System.nanoTime();
		long time;

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
				this.setBothSpeed(180);

				// 4.5: continue on to next iteration of big loop
				continue;
			} else if (this.leftBump()) {
				System.out.println("left bump");
				this.setDiffSpeeds(180, 270);
				continue;
			}

			if (sonic <= .05) {
				startTime = System.nanoTime();
				this.setDiffSpeeds(270, 180);
				while (sonic <= .05) {
					sonic = sonicSense();
				}
				time = (System.nanoTime() - startTime);
				this.turnRight(270, 180, time, getAngle(270, 180, time));
				this.currentPosition.printPosition();
				// Option 1: way too close -- move to the right fast
			} else if (sonic < .12) {
				// Option 2: way too close -- move to the right fast
				startTime = System.nanoTime();
				this.setDiffSpeeds(240, 180);
				while (sonic < .12) {
					sonic = sonicSense();
				}
				time = (System.nanoTime() - startTime);
				this.turnRight(240, 180, time, getAngle(240, 180, time));
				this.currentPosition.printPosition();
			} else if (sonic < .14) {
				// Option 3: a little to close to wall -- adjust right a little
				startTime = System.nanoTime();
				this.setDiffSpeeds(210, 180);
				while (sonic < .14) {
					sonic = sonicSense();
				}
				time = (System.nanoTime() - startTime);
				this.turnRight(210, 180, time, getAngle(210, 180, time));
				this.currentPosition.printPosition();
			} else if (sonic >= .14 && sonic <= .16) {
				// Option 4: just right -- set wheels to same speed and move on forward
				startTime = System.nanoTime();
				this.setBothSpeed(180);
				while (sonic >= .14 && sonic <= .16) {
					sonic = sonicSense();
				}
				time = (System.nanoTime() - startTime);
				this.getPositionStraight(this.heading, 180, time);
				this.currentPosition.printPosition();
			} else if (sonic > .16 && sonic <= .2) {
				// Option 5: a little too far from wall -- move left slow
				startTime = System.nanoTime();
				this.setDiffSpeeds(180, 210);
				while (sonic > .16 && sonic <= .2) {
					sonic = sonicSense();
				}
				time = (System.nanoTime() - startTime);
				this.turnLeft(180, 210, time, getAngle(180, 210, time));
				this.currentPosition.printPosition();
			} else if (sonic > .2 && sonic <= .25) {
				// Option 6: a little too far from wall -- move left slow
				startTime = System.nanoTime();
				this.setDiffSpeeds(180, 240);
				while (sonic > .2 && sonic <= .25) {
					sonic = sonicSense();
				}
				time = (System.nanoTime() - startTime);
				this.turnLeft(180, 240, time, getAngle(180, 240, time));
				this.currentPosition.printPosition();
			} else { // .2
				// Option 7: way too far -- move to the left fast
				startTime = System.nanoTime();
				this.setDiffSpeeds(180, 270);
				while (sonic > .25 && sonic <= .4) {
					sonic = sonicSense();
				}
				time = (System.nanoTime() - startTime);
				this.turnLeft(180, 270, time, getAngle(180, 270, time));
				this.currentPosition.printPosition();
			}
			sonic = this.sonicSense();
//			if (sonic > .4) {
////					this.motorM.rotateTo((int) this.heading);
////					float sonic1 = this.sonicSense();
////					this.motorM.rotate(45);
////					float sonic2 = this.sonicSense();
//				System.out.println("in greater than .4 statement " + sonic);
//				this.stopBothInstant();
////					this.rotateLeft(30);
//				sonic = sonicSense();
//				if (sonic > .5) {
//					break;
//				}
//			}
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
			sonic.fetchSample(sample_sonic, 0);
			dists[i] = sample_sonic[0];
			int loop = 0;
			while (dists[i] > .8 && loop < 10) {
				System.out.println("Infinity " + loop);
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

//		public void moveToward(double x, double y) {
	//
//		}

	/*
	 * Name: rotateRight in: degrees to rotate out: nothing description: should turn
	 * the robot in place towards goal
	 */

	public void rotateRight(long degrees) {
		this.stopSync();
		this.setLeftSpeed(180);
		this.motorL.forward();

		// long delay= (long) (degrees/av)*1000; // need to set delay time
		long delay = this.timeToRotate(0, 180, degrees);
		Delay.msDelay(delay);

		// stop both motors
		this.stopBothInstant();
		this.turnRight(0, 180, (long) (delay * Math.pow(10, 9)), -degrees);

	}

	/*
	 * Name: rotateLeft in: degrees to rotate out: nothing description: should turn
	 * the robot in place towards goal
	 */

	public void rotateLeft(int degrees) {
		// move right forward and left backward to create a spin
		this.stopSync();
		this.setRightSpeed(180);

		this.motorR.forward();

		long delay = this.timeToRotate(180, 0, degrees);
		Delay.msDelay(delay);

		// stop both motors
		this.stopBothInstant();
		this.turnLeft(180, 0, (long) (delay * Math.pow(10, 9)), degrees);
	}

//		public float theta() {
//			float[] sample_gyro = new float[gyro.sampleSize()];
//			this.gyro.fetchSample(sample_gyro, 0);
//			return sample_gyro[0];
//		}

	public void setHeading(double theta) {
		this.heading = theta;
	}

	public double getHeading() {
		return this.heading;
	}

	public Coordinate getPosition() {
		return this.currentPosition;
	}

//		/*Name: timeToRotate
//		 * in: ul, ur, theta
//		 * out: seconds to move
//		 * */
	//
	public long timeToRotate(double ur, double ul, double theta) {
		double vr = ur * Math.PI / 180 * this.radiusR;
		double vl = ul * Math.PI / 180 * this.radiusL;
		double omega = (vr - vl) / this.L;
		double time = (theta * Math.PI / 180) / omega;
		return (long) time * 1000;
	}

	public double getAngle(double ur, double ul, long time) {
		double vr = ur * Math.PI / 180 * this.radiusR;
		double vl = ul * Math.PI / 180 * this.radiusL;
		double omega = (vr - vl) / this.L;
		System.out.println("Omega = " + omega);
		double theta = omega * time / Math.pow(10, 9);
		System.out.println(theta * 180 / Math.PI);
		return theta * 180 / Math.PI;
	}

	public void turnRight(double ur, double ul, long time, double degrees) {
		double vr = ur * Math.PI / 180 * this.radiusR;
		double vl = ul * Math.PI / 180 * this.radiusL;
		double R = (this.L / 2) * ((vl + vr) / (vl - vr));
		double angle = degrees * Math.PI / 180;
//			Coordinate ICC = new Coordinate(this.currentPosition.getX() + R * Math.cos(angle),
//					this.currentPosition.getY() - R * Math.sin(angle));
		Coordinate ICC = new Coordinate(this.currentPosition.getX() + R * Math.sin(angle),
				this.currentPosition.getY() - R * Math.cos(angle));
		this.currentPosition = getPositionClockwise(angle, ICC);
		this.heading += degrees;
	}

	public void turnLeft(double ur, double ul, long time, double degrees) {
		double vr = ur * Math.PI / 180 * this.radiusR;
		double vl = ul * Math.PI / 180 * this.radiusL;
		double R = (this.L / 2) * ((vl + vr) / (vr - vl));
		double angle = degrees * Math.PI / 180;
//			Coordinate ICC = new Coordinate(this.currentPosition.getX() - R * Math.cos(angle),
//					this.currentPosition.getY() + R * Math.sin(angle));
		Coordinate ICC = new Coordinate(this.currentPosition.getX() - R * Math.sin(angle),
				this.currentPosition.getY() + R * Math.cos(angle));
		this.currentPosition = getPositionCCwise(angle, ICC);
		this.heading += degrees;
	}

	public Coordinate getPositionClockwise(double angle, Coordinate ICC) {
		double ICCx = ICC.getX();
		double ICCy = ICC.getY();
		angle *= -1;
		double x = (Math.cos(angle) * (this.currentPosition.getX() - ICCx))
				- (Math.sin(angle) * (this.currentPosition.getY() - ICCy)) + ICCx;
//			double x = (Math.sin(angle) * (this.currentPosition.getY() - ICCy))
//					- (Math.cos(angle) * (this.currentPosition.getX() - ICCx)) + ICCx;
		double y = (Math.sin(angle) * (this.currentPosition.getX() - ICCx))
				+ (Math.cos(angle) * (this.currentPosition.getY() - ICCy)) + ICCy;
		return new Coordinate(x, y);
	}

	public Coordinate getPositionCCwise(double angle, Coordinate ICC) {
		double ICCx = ICC.getX();
		double ICCy = ICC.getY();
		double x = (Math.cos(angle) * (this.currentPosition.getX() - ICCx))
				- (Math.sin(angle) * (this.currentPosition.getY() - ICCy)) + ICCx;
		double y = (Math.sin(angle) * (this.currentPosition.getX() - ICCx))
				+ (Math.cos(angle) * (this.currentPosition.getY() - ICCy)) + ICCy;
		return new Coordinate(x, y);
	}

	public void getPositionStraight(double degrees, float u, long time) {
		double v = u * Math.PI / 180 * this.radiusR;
		double x = this.currentPosition.getX() + (v * Math.cos(degrees * Math.PI / 180) * (time / Math.pow(10, 9)));
		double y = this.currentPosition.getY() + (v * Math.sin(degrees * Math.PI / 180) * (time / Math.pow(10, 9)));
		this.currentPosition = new Coordinate(x, y);
	}

	public void goAtSetSpeed(float ur, float ul) {
		this.setDiffSpeeds(ul, ur);
		this.moveForwardBoth();
		long time = (long) (5 * Math.pow(10, 9));
		this.stopBothInstant();
		if (ur > ul) {
			System.out.println("R>L");
			turnLeft(ur, ul, time, getAngle(ur, ul, time));
			System.out.println(this.heading);
		} else if (ur < ul) {
			System.out.println("R<L");
			turnRight(ur, ul, time, getAngle(ur, ul, time));
			System.out.println(this.heading);
		} else {
			System.out.println("R=L");
			getPositionStraight(this.heading, ur, time);
			System.out.println(this.heading);
		}
		this.currentPosition.printPosition();
	}

	///////////// Sarah's new methods:
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

}
