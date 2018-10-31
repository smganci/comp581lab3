package lab3;

/*Collaborators:
 * Tricia Bacon (730011125)
 * Sarah Ganci (720510446)
 * 
 * */

public class Lab3 {

	public static void main(String[] args) {
		Charlie charlie = new Charlie();

		// 1: press button to start
		System.out.println("Press Button to Start");
		charlie.buttonWait();

		// 2: move toward 20 cm away
		charlie.moveTillSense(.20);

		// 3: beep within 30 cm
		charlie.beep();

		// 4: turn sensor
		charlie.rotateSonic(-70);

		// 5: rotate charlie right
		charlie.rotateRight(90);

		// 6: trace wall
		charlie.trace();

		// 7: return to start
		charlie.returnToStart();

		// 8: Happy Halloween :)
		charlie.thisIsHalloween();

	}

}
