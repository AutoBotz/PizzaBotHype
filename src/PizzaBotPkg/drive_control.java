package PizzaBotPkg;
import lejos.utility.Stopwatch;
import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.utility.Delay;

//import statements

/**
* @author      Andrej Janda, Ethan Waldie, Michael Ding
* @version     0.1
* @since       0.0
*/

public class drive_control {
	public static float Lwheel_amt_per_cm = Float.NaN;
	public static float Lwheel_amt_full_rotation = Float.NaN;
	public static float Lwheel_amt_pivot_turn = Float.NaN;
	public static float Lwheel_distance_full_rev = Float.NaN;
	public static float Lwheel_center = Float.NaN;

	public static float Rwheel_amt_per_cm = Float.NaN;
	public static float Rwheel_amt_full_rotation = Float.NaN;
	public static float Rwheel_amt_pivot_turn = Float.NaN;
	public static float Rwheel_distance_full_rev = Float.NaN;
	public static float Rwheel_center = Float.NaN;

	public static float pi = (float) Math.PI;


	public static void set_dims(float left_diameter, float right_diameter, float wheel_base, float wheel_width){
		/**
		 * This function accept the physical dimensions of the robot, and computes the corrections factors
		 * for functions such as rotation, turn and forward driving to allow user to input reasonable numbers
		 * into the control functions. Such as centimeters and centimeters per second.
		 *
		 * @param left_diameter Diameter of the left wheel
		 * @param right_diameter Diameter of the right wheel
		 * @param wheel_base The inner distance between two wheels
		 * @param wheel_width The width of the wheels
		 *
		 */

		Lwheel_distance_full_rev = pi * (left_diameter/2) * (left_diameter/2);
		Rwheel_distance_full_rev = pi * (right_diameter/2) * (right_diameter/2);

		Lwheel_amt_per_cm = 360/Lwheel_distance_full_rev;
		Rwheel_amt_per_cm = 360/Rwheel_distance_full_rev;

		Lwheel_center = (wheel_base/2) + (wheel_width/2);
		Rwheel_center = (wheel_base/2) + (wheel_width/2);

		Lwheel_amt_full_rotation = (wheel_base + wheel_width)  * pi



	}


	public static void forward(int distance){
		 double A_ang = Motor.A.getTachoCount();
		 double B_ang = Motor.B.getTachoCount();

		 A_ang = A_ang + distance * (360/17);
		 B_ang = B_ang + distance * (360/17);

		 Motor.A.rotateTo((int)(A_ang), true);
		 Motor.B.rotateTo((int)(B_ang));
	}

	public static void spotTurn(int angturn){
		 int A_ang = Motor.A.getTachoCount();
		 int B_ang = Motor.B.getTachoCount();
		 int turnAmt = angturn ;


		 A_ang = A_ang - turnAmt;
		 B_ang = B_ang + turnAmt;

		 Motor.A.rotateTo(A_ang, true);
		 Motor.B.rotateTo(B_ang);

	}
	static void turn(int nominator, int denominator, int Speed, int direction) {
		  if (direction > 0)
			  Motor.A.setSpeed(denominator*Speed);
		  	  Motor.B.setSpeed(nominator*Speed);
		  	  Motor.A.forward();
		  	  Motor.B.forward();


	  	if (direction < 0)
	  		Motor.A.setSpeed(nominator*Speed);
	  		Motor.B.setSpeed(denominator*Speed);
	  		Motor.A.forward();
	  		Motor.B.forward();
	 }


	public static void set_speed(int a, int b) {
		   Motor.A.setSpeed(a);
		   Motor.B.setSpeed(b);
	}
}
