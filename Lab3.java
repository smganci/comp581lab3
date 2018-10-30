package lab3;

/*Collaborators:
 * Tricia Bacon (730011125)
 * Sarah Ganci (720510446)
 * 
 * */

public class Lab3 {

	public static void main(String[] args) {
		Charlieold charlie = new Charlieold();

		// 1: press button to start
		System.out.println("Press Button to Start");
		charlie.buttonWait();

		// 2: move toward 20 cm away and beep
		charlie.moveTillSense(.20);

		// System.out.println("the original goal dist was: " + goal_dist);
		// charlie.buttonWait();
		// // charlie.beep();

		// 4: turn sensor
		charlie.rotateSonic(-75);

		// 3: rotate charlie right
		charlie.rotateRight(105);

		// 5: begin trace
		System.out.println("Trace 3 begins");
		charlie.trace3();

		// 6: print out original distance
		// System.out.println("the original goal dist was: " + goal_dist);

		charlie.buttonWait();

		// should test rotate left till sense
//		charlie.rotateLeftTillSense(.5);
		// should test new version of trace

	}

}
