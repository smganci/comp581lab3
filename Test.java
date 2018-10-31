package lab3;

public class Test {

	public static void main(String[] args) {

		Charlie charlie = new Charlie();
		System.out.println("press to start");
		charlie.buttonWait();

		charlie.moveTillTouch();
		charlie.moveBackwardDist(.15);

	}
}
