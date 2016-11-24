package PizzaBotPkg;
import lejos.utility.Stopwatch;
import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.utility.Delay;

//import statements

/**
* @author      Andrej Janda, Ethan Waldie, Michael Ding
* @version     0.1
* @since       0.0
*/

public class drive_control {
	public float Lwheel_amt_per_cm = Float.NaN;
	public float Lwheel_amt_full_rotation = Float.NaN;
	public float Lwheel_amt_pivot_turn = Float.NaN;
	public float Lwheel_distance_full_rev = Float.NaN;
	public float Lwheel_center = Float.NaN;

	public float Rwheel_amt_per_cm = Float.NaN;
	public float Rwheel_amt_full_rotation = Float.NaN;
	public float Rwheel_amt_pivot_turn = Float.NaN;
	public float Rwheel_distance_full_rev = Float.NaN;
	public float Rwheel_center = Float.NaN;

	public static double pi = Math.PI;

	public double X ;
	public double Y ;

	// Programmatics
	public static EV3GyroSensor gyro;
	public static EV3UltrasonicSensor sonic;


	public float[] gyro_sample;
	public float[] sonicsample;


	// Global statements
	// A is left wheel
	// B is right wheel
	// A,B forward make robot go forward
	public void set_dims(float left_diameter, float right_diameter, float wheel_base){
		/**
		 * This function accept the physical dimensions of the robot, and computes the corrections factors
		 * for functions such as rotation, turn and forward driving to allow user to input reasonable numbers
		 * into the control functions. Such as centimeters and centimeters per second.
		 *
		 * Returns nothing
		 *
		 * @param left_diameter Diameter of the left wheel
		 * @param right_diameter Diameter of the right wheel
		 * @param wheel_base The inner distance between two wheels
		 * @param wheel_width The width of the wheels
		 */

		Lwheel_distance_full_rev = (float) pi * (left_diameter/2) * (left_diameter/2);
		Rwheel_distance_full_rev = (float) pi * (right_diameter/2) * (right_diameter/2);

		Lwheel_amt_per_cm = 360/Lwheel_distance_full_rev;
		Rwheel_amt_per_cm = 360/Rwheel_distance_full_rev;


	}


	public void init_pos(double x_init, double y_init){
		X = x_init;
		Y = y_init;
	}
	public void forward_with_sonic(int distance, int speed){
	}


	public void forward(int distance, int speed){
		/**
		 * This function handles driving forward of robot
		 *
		 * Returns nothing
		 *
		 * @param distance Distance that robot should travel, in centimeters
		 * @param speed speed of the wheels in centimeter per second
		 */

		this.set_speed(speed, speed);

		double angle = (double)theta() %360;
		angle = (angle/360) * 2 * pi;

		double x = distance*Math.sin(angle);
		double y = distance*Math.cos(angle);

		X += x;
		Y += y;

		double A_ang = Motor.A.getTachoCount();
		double B_ang = Motor.B.getTachoCount();

		A_ang = A_ang + distance * (Lwheel_amt_per_cm);
		B_ang = B_ang + distance * (Rwheel_amt_per_cm);

		Motor.A.rotateTo((int)(A_ang), true);
		Motor.B.rotateTo((int)(B_ang));
	}

	public void spotTurn(int angturn, int speed){
		/**
		 * This function let robot rotate without moving translationally.
		 * Wheel odometry is used for counting of rotation
		 *
		 * Returns nothing
		 *
		 * @param angturn Angles of rotation desired, in degrees. +ve is CW, -vs is CCW
		 * @param speed speed of the wheels in centimeter per second
		 */

		 this.set_speed(speed, speed);
		 int A_ang = Motor.A.getTachoCount();
		 int B_ang = Motor.B.getTachoCount();
		 int turnAmt = angturn ;


		 A_ang = A_ang - turnAmt;
		 B_ang = B_ang + turnAmt;

		 Motor.A.rotateTo(A_ang, true);
		 Motor.B.rotateTo(B_ang);
	}

	public void spotTurn_gyro(int angturn){		/**
		 * This function let robot rotate without moving translationally.
		 * Gyroscope angle reading is used for counting of rotation
		 *
		 * Returns nothing
		 *
		 * @param angturn Angles of rotation desired, in degrees. +ve is CW, -vs is CCW
		 * @param speed speed of the wheels in centimeter per second
		 */
			double K = 0.8;
			double angGoal =  angturn; // Determine the goal angle to turn to
			while (Math.abs(theta() - angGoal) > 0.5) {
				double speed = K * Math.abs(this.theta() - angGoal);
				set_speed((int)(speed+50), (int)(speed+50));

				if ((theta() - angGoal) > 0) {
					Motor.A.backward();
					Motor.B.forward();
				} else {
					Motor.A.forward();
					Motor.B.backward();
				}
			}

			this.flt();
		}

	public void turn(int nominator, int denominator, int Speed, int direction) {
		/**
		 * This function let robot turn with a given radius of turn
		 *
		 * Returns nothing
		 *
		 * @param nominator speed scaling for left wheel
		 * @param denominator speed scaling for right wheel
		 * @param Speed speed of the wheels in centimeter per second
		 * @param direction positive for forward, negative for back
		 */
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


	public void set_speed(float a, float b) {
		/**
		 * This function set speed of the wheels of the robot
		 *
		 * Returns nothing
		 *
		 * @param a speed of the left wheel, in cemtimeters per second
		 * @param b speed of the left wheel, in cemtimeters per second
		 */
		   Motor.A.setSpeed((int) a);
		   Motor.B.setSpeed((int) b);
	}

	public float smooth_theta(){
		double sum = 0.0;
		for (int i = 0; i < 10; i++)
			sum += this.theta();

		return (float) sum / 10;
	}

	public void flt() {
			Motor.A.flt(true);
			Motor.B.flt();
	}

	public void stop() {
			Motor.A.stop(true);
			Motor.B.stop();
	}

	public void gyro_init(int portNum){
		if (portNum == 1){
			gyro = new EV3GyroSensor(SensorPort.S1);
		} else if (portNum == 2) {
			gyro = new EV3GyroSensor(SensorPort.S2);
		} else if (portNum == 3) {
			gyro = new EV3GyroSensor(SensorPort.S3);
		} else if (portNum == 4) {
			gyro = new EV3GyroSensor(SensorPort.S4);
		}

		gyro.getAngleMode(); 		// Set to purely angle mode
		int gyro_sample_size = gyro.sampleSize(); //Modify gyro sample buffer to account of change of mode
		gyro_sample = new float[gyro_sample_size];

		this.gyro_cal(); // Call gyro calibration to recalibrate gyro
	}

	public void gyro_cal() {
		this.stop(); // Full stop, robot must be stationary for gyro calibration
		System.out.println("Hold for gyro calibration");
		Delay.msDelay(500);
		gyro.reset(); 					// Reset the gyro

		// Wait for gyro to finish calibrating
		// will output NaN until calibration complete
		while (theta() == Float.NaN){
			Delay.msDelay(40);
		}

		System.out.println("Gyro calibration complete");
	}

	public void sonic_init(int portNum){
		if (portNum == 1){
			sonic = new EV3UltrasonicSensor(SensorPort.S1);
		} else if (portNum == 2) {
			sonic = new EV3UltrasonicSensor(SensorPort.S2);
		} else if (portNum == 3) {
			sonic = new EV3UltrasonicSensor(SensorPort.S3);
		} else if (portNum == 4) {
			sonic = new EV3UltrasonicSensor(SensorPort.S4);
		}

		int sonic_sampleSize = sonic.sampleSize();
		sonicsample = new float[sonic_sampleSize];
	}

	public float theta() {
		gyro.getAngleMode().fetchSample(gyro_sample,0);
		return gyro_sample[0] % 360;
	}

	public float ping(){
		/**
		 * This function returns the distance read by ultrasonic sensor
		 */
		sonic.fetchSample(sonicsample, 0);
		return sonicsample[0]*100;

	}

	public float avg_ping(){

		// Each ping should take 20ms to call, so 5 average will take 100ms.

		double sum = 0.0;


		for (int i = 0; i < 5; i++)
			sum += this.ping();

		return (float) sum / 5;
	}

	public void getCoordinate(int angturn, int speed){
		 set_speed(speed, speed);
		 int A_ang = Motor.A.getTachoCount();
		 int B_ang = Motor.B.getTachoCount();
		 int turnAmt = angturn ;


		 A_ang = A_ang - turnAmt;
		 B_ang = B_ang + turnAmt;

		 Motor.A.rotateTo(A_ang, true);
		 Motor.B.rotateTo(B_ang);

	}

	public float[] sweep_ping(){
		/**
		 * Start the ultrasonic sensor at the center
		 *
		 * distance array representation
		 *
		 *    0        1          2         3        4         5          6          7        8
		 * [ Left ] [ FLL ] [ Fwd Left ] [ FFL ] [ Center ] [ FFR ] [ Fwd Right ] [ FRR ] [ Right ]
		 */
		float [] distance_array = new float[9];
		Motor.C.setSpeed(60);
		Motor.C.rotateTo(-90);
		for (int i = 0; i < 9; i ++){
			Motor.C.rotateTo((int)(i*22.5 - 90));
			distance_array[i] = this.avg_ping();
		}
		return distance_array;
	}
}
