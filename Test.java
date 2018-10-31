package lab3;

public class Test {

	public static void main(String[] args) {

		Charlie charlie = new Charlie();
		charlie.printPos();
		System.out.println("press to start");
		charlie.buttonWait();

		charlie.moveTillTouch();
		charlie.printPos();
		charlie.buttonWait();

		charlie.moveBackwardDist(.15);

		charlie.printPos();
		charlie.buttonWait();

		charlie.buttonWait();
		charlie.rotateRight(105);
		charlie.printPos();
		charlie.buttonWait();

	}
}
