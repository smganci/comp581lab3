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

		// testing sonic rotateSense()

		System.out.println("Press Button to Start");
		Button.ENTER.waitForPressAndRelease();
		charlie.buttonWait();
		float[] sonicsensed = charlie.sonicRotateSense();
		for (int i = 0; i < sonicsensed.length; i++) {
			System.out.println(sonicsensed[i]);
		}
		charlie.buttonWait();

	}

}
