package lab3;

import lejos.hardware.Button;

/*Collaborators:
 * Tricia Bacon (730011125)
 * Sarah Ganci (720510446) 
 */

public class Lab3 {

	public static void main(String[] args) {

		// Set up Charlie
		Charlie charlie = new Charlie();

		// Wait to start
		System.out.println("Lab 3");
		System.out.println("Press Button to Start");
		Button.ENTER.waitForPressAndRelease();

		// TODO: Determine (x,y) coordinate and heading
		charlie.getPosition().printPosition();

		// Move towards wall and beep when at hit point
		charlie.moveTillSense(.30);
		charlie.getPosition().printPosition();
		charlie.beep();
		Button.ENTER.waitForPressAndRelease();

		// turn robot right
		charlie.rotateRight(90);

		// turn sensor
		charlie.rotateSonic(-90);
		charlie.getPosition().printPosition();
		Button.ENTER.waitForPressAndRelease();

		// follow wall until back at hit point
//		charlie.goAtSetSpeed(180, 270);
//		charlie.goAtSetSpeed(270, 180);
//		charlie.goAtSetSpeed(180, 180);
//		Button.ENTER.waitForPressAndRelease();

		// turn robot right again

		// TODO: return to start point

	}

}
