package lab3;

import lejos.hardware.Button;

/*Collaborators:
 * Tricia Bacon (730011125)
 * Sarah Ganci (720510446)
 * 
 * */

public class Lab3 {

	public static void main(String[] args) {
		Charlie charlie = new Charlie();

//		/// testing touch
//		while (!Button.ENTER.isDown()) {
//
//			System.out.println("Touch Left: " + charlie.leftBump());
//			System.out.println("Touch Front: " + charlie.frontBump());
//
//			Delay.msDelay(750);// Just so it does not spam output
//		}

//		// testing sonic rotateSense()
//
//		System.out.println("Press Button to Start");
//		Button.ENTER.waitForPressAndRelease();
//		charlie.buttonWait();
//
		System.out.println("Press enter to start");
		Button.ENTER.waitForPressAndRelease();
		charlie.moveTillSense(.20);
		charlie.rotateRight(105);
//
//		// 3: turn sensor
		charlie.rotateSonic(-90);

		System.out.println("Trace 2 begins");
		charlie.trace2();
//
//		System.out.println("fin");
//
		charlie.buttonWait();

		// should test rotate left till sense
//		charlie.rotateLeftTillSense(.5);
		// should test new version of trace

	}

}
