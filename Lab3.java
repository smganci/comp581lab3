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

		// Move towards wall and beep when at hit point
		charlie.moveTillSense(.20);
		charlie.beep();

		// turn robot right
		charlie.rotateRight(105);

		// turn sensor
		charlie.rotateSonic(-90);
		// Button.ENTER.waitForPressAndRelease();

		// follow wall until back at hit point
		charlie.trace();

		// turn robot right again
		charlie.rotateRight(105);

		// TODO: return to start point

	}

}
