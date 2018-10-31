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

		// 2: move toward 20 cm away and beep
		charlie.setBothSpeed(270);
		charlie.moveTillSense(.20);

		charlie.beep();

		// 4: turn sensor
		charlie.rotateSonic(-70);

		// 3: rotate charlie right
		charlie.rotateRight(90);

		System.out.println("Current pos is:");
		charlie.printPos();
		System.out.println("Current gyro reading is:" + charlie.gthetha());
		charlie.buttonWait();

		// 5: begin trace
		System.out.println("Trace 3 begins");
		charlie.trace();

		System.out.println("Current pos is:");
		charlie.printPos();
		System.out.println("ending gyro reading is:" + charlie.gthetha());
		charlie.buttonWait();
		charlie.returnToStart();

		charlie.buttonWait();
		// charlie.thisIsHalloween();

	}

}
