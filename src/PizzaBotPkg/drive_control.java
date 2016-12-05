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
	public double Lwheel_amt_per_cm = Float.NaN;
	public double Lwheel_amt_full_rotation = Float.NaN;
	public double Lwheel_amt_pivot_turn = Float.NaN;
	public double Lwheel_distance_full_rev = Float.NaN;
	public double Lwheel_center = Float.NaN;

	public double Rwheel_amt_per_cm = Float.NaN;
	public double Rwheel_amt_full_rotation = Float.NaN;
	public double Rwheel_amt_pivot_turn = Float.NaN;
	public double Rwheel_distance_full_rev = Float.NaN;
	public double Rwheel_center = Float.NaN;

	public static float house_distance = 10; // WE NEED TO MEASURE THIS
	public static int house_count = 0;
	public static int house_edge = 0;
	public static int house_buffer_count = 0;

	public static double pi = Math.PI;

	public double X = 0.0;
	public double Y = 0.0;

	// Programmatics
	public static EV3GyroSensor gyro;
	public static EV3UltrasonicSensor sonic;


	public float[] gyro_sample;
	public float[] sonicsample;



	// Global statements
	// A is left wheel
	// B is right wheel
	// A,B forward make robot go forward
	public void set_dims(double left_diameter, double right_diameter, double wheel_base){
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

		Lwheel_distance_full_rev = (double) pi * (left_diameter/2) * (left_diameter/2);
		Rwheel_distance_full_rev = (double) pi * (right_diameter/2) * (right_diameter/2);

		Lwheel_amt_per_cm = 360/Lwheel_distance_full_rev;
		Rwheel_amt_per_cm = 360/Rwheel_distance_full_rev;


	}

	public void init_pos(double x_init, double y_init){
		X = x_init;
		Y = y_init;
	}

	public void move_to_Point_PID_SONIC(double x, double y, int speed){
		// Theta in radians
		set_speed(speed, speed);

		// Set the current heading as the desired orientation
		double angle = desired_Orientation(x,y);
		System.out.println("ORIGINAL ANGLE " + angle);
		double distance = 0;
		spotTurn_gyro((int)angle);
		double error = (double)(theta() - angle);

		// proportional constants
		double kp = 0.8;

		System.out.println("(" + X + "," + Y + ")");
		double atacho = Motor.A.getTachoCount();
		double btacho = Motor.B.getTachoCount();



		// While loop for constant
		while (true){
			if (Math.abs(X - x) < 3 && Math.abs(Y - y) < 3) break;
			
			System.out.println("(" + X + "," + Y + ")");
			if (Button.ESCAPE.isDown()) {
		    	Motor.A.flt();
		    	Motor.B.flt();
		    	Delay.msDelay(1000);
		       	if (Button.ESCAPE.isDown()){break;}
		    	}


			// If there is an obstacle, call avoidance routine
			if (avg_ping() < 15){
				//object_avoid();
				object_avoid_follow(speed);
				angle = desired_Orientation(x,y);
				spotTurn_gyro((int)angle);
				System.out.println("NEW ANGLE " + angle);
				atacho = Motor.A.getTachoCount();
				btacho = Motor.B.getTachoCount();
			}

			error = (double)(theta() - angle);
			double diff = kp * error;

			if (diff > speed ){
				diff = speed;
			}
			//System.out.println(diff);

			// Update based on PID
			set_speed((int)(speed + diff), (int)(speed - diff));
			Motor.A.forward();
			Motor.B.forward();

			// Get tacho counts
			distance = ((Motor.A.getTachoCount() - atacho) + (Motor.B.getTachoCount() - btacho))*0.35/Rwheel_amt_per_cm;
			atacho = Motor.A.getTachoCount();
			btacho = Motor.B.getTachoCount();
			System.out.println(distance);
			
			//System.out.println(distance);
			// Update position traveled
			X += distance*Math.sin(Math.toRadians(angle));
			Y += distance*Math.cos(Math.toRadians(angle));
		}
		stop();
	}
	public void move_to_house(double x, double y, int speed, int house_num){
		// Theta in radians
		set_speed(speed, speed);

		// Set the current heading as the desired orientation
		double angle = desired_Orientation(x,y);
		System.out.println("ORIGINAL ANGLE " + angle);
		double distance = 0;
		spotTurn_gyro((int)angle);
		double error = (double)(theta() - angle);

		// proportional constants
		double kp = 0.8;

		//System.out.println("(" + X + "," + Y + ")");
		double atacho = Motor.A.getTachoCount();
		double btacho = Motor.B.getTachoCount();

		int loop_count = 0;

		// While loop for constant
		while (true){
			if (Math.abs(X - x) < 3 && Math.abs(Y - y) < 3) break;
			
			//System.out.println("(" + X + "," + Y + ")");

			if (loop_count == 0){
	    		System.out.print(count_house()+"    ");
	    		System.out.println(ping());
			}
			loop_count += 1;
			loop_count = loop_count %20;
			
			if (Button.ESCAPE.isDown()) {
		    	Motor.A.flt();
		    	Motor.B.flt();
		    	Delay.msDelay(1000);
		       	if (Button.ESCAPE.isDown()){break;}
		    	}
			// If house is found, drop pizzza
			if (count_house() == house_num){
				stop();
				distance = ((Motor.A.getTachoCount() - atacho) + (Motor.B.getTachoCount() - btacho))*0.35/Rwheel_amt_per_cm;
				System.out.println(distance);
				
				X += distance*Math.sin(Math.toRadians(angle));
				Y += distance*Math.cos(Math.toRadians(angle));
				
				drop_pizza(house_num);
				
				break;
			}

			error = (double)(theta() - angle);
			double diff = kp * error;

			if (diff > speed ){
				diff = speed;
			}
			//System.out.println(diff);

			// Update based on PID
			set_speed((int)(speed + diff), (int)(speed - diff));
			Motor.A.forward();
			Motor.B.forward();

			// Get tacho counts
			distance = ((Motor.A.getTachoCount() - atacho) + (Motor.B.getTachoCount() - btacho))*0.35/Rwheel_amt_per_cm;
			atacho = Motor.A.getTachoCount();
			btacho = Motor.B.getTachoCount();
			System.out.println(distance);
			
			//System.out.println(distance);
			// Update position traveled
			X += distance*Math.sin(Math.toRadians(angle));
			Y += distance*Math.cos(Math.toRadians(angle));
		}
		stop();
	}

	public void reverse_to_Point_PID(double x, double y, int speed){
		// Theta in radians
		set_speed(speed, speed);

		// Set the current heading as the desired orientation, reverse
		double angle = (desired_Orientation(x,y)+pi)%(2*pi) ;

		System.out.println("ORIGINAL ANGLE " + angle);
		double distance = 0;
		spotTurn_gyro((int)angle);
		double error = (double)((theta()+pi)%(2*pi) - angle);

		// Integral and derivative terms
		double integral = 0;
		double derivative = 0;

		// proportional constants
		double kp = 0.8;

		System.out.println("(" + X + "," + Y + ")");

		// While loop for constant
		while (true){
			if (Math.abs(X - x) < 3 || Math.abs(Y - y) < 3) break;

			System.out.println("(" + X + "," + Y + ")");
			if (Button.ESCAPE.isDown()) {
		    	Motor.A.flt();
		    	Motor.B.flt();
		    	Delay.msDelay(1000);
		       	if (Button.ESCAPE.isDown()){break;}
		    	}

			int atacho = Motor.A.getTachoCount();
			int btacho = Motor.B.getTachoCount();

			error = (double)(theta() - angle);
			double diff = kp * error;

			if (diff > speed ){
				diff = speed;
			}
			//System.out.println(diff);

			// Update based on PID
			set_speed((int)(speed + diff), (int)(speed - diff));
			Motor.A.backward();
			Motor.B.backward();

			// Get tacho counts
			distance = ((Motor.A.getTachoCount() - atacho) + (Motor.B.getTachoCount() - btacho))*0.5/Rwheel_amt_per_cm;

			//System.out.println(distance);
			// Update position traveled
			X += distance*Math.sin(Math.toRadians(angle));
			Y += distance*Math.cos(Math.toRadians(angle));
		}
	}

	public void object_avoid(){
		// Avoid object by turning right and
		// traveling 20 cm
		System.out.println("(" + (int)X + ", " + (int)Y +")");
		spotTurn_gyro((int)(theta() + 90));
		forward(20, 150);
		System.out.println("(" + (int)X + ", " + (int)Y +")");

	}


	public void object_avoid_follow(float speed){
		// Avoid object by turning right and
		// traveling 20 cm
		System.out.println("(" + (int)X + ", " + (int)Y +")");
		spotTurn_gyro((int)(theta() + 90));
		Motor.C.rotateTo(-90);
		// double prev_ping = avg_ping();
		double wall_sep = avg_ping();

		// Theta in radians
		set_speed(speed, speed);

		// Integral and derivative terms
		double integral = 0;
		double derivative = 0;
		double error = 0;
		double distance = 0;

		// proportional constants
		double kp = 10;

		System.out.println("(" + X + "," + Y + ")");

		double atacho = Motor.A.getTachoCount();
		double btacho = Motor.B.getTachoCount();

		// While loop for constant
		while (true){
			if (Math.abs(avg_ping() - wall_sep) > 10){break;}

			System.out.println("(" + X + "," + Y + ")");
			if (Button.ESCAPE.isDown()) {
		    	Motor.A.flt();
		    	Motor.B.flt();
		    	Delay.msDelay(1000);
		       	if (Button.ESCAPE.isDown()){break;}
		    	}

			error = (double)(avg_ping() - wall_sep);
			double diff = kp * error;

			if (diff > speed ){
				diff = speed;
			}
			//System.out.println(diff);

			// Update based on PID
			set_speed((int)(speed + diff), (int)(speed - diff));
			Motor.A.forward();
			Motor.B.forward();

			// Get tacho counts
			distance = ((Motor.A.getTachoCount() - atacho) + (Motor.B.getTachoCount() - btacho))*0.37/Rwheel_amt_per_cm;
			atacho = Motor.A.getTachoCount();
			btacho = Motor.B.getTachoCount();
			System.out.println(distance);
			
			//System.out.println(distance);
			// Update position traveled
			double angle = theta();
			X += distance*Math.sin(Math.toRadians(angle));
			Y += distance*Math.cos(Math.toRadians(angle));
		}

		stop();
		this.forward(20,200);
		this.spotTurn_gyro(theta()-90);
		this.forward(20,200);
		Motor.C.rotateTo(0);

	}

	public double desired_Orientation (double x, double y){
		// Set angle taking into account boundary cases

		double angle = Math.toDegrees(Math.atan((x-X)/(y-Y)));

		// straight up and down x axis
		if (y-Y==0){
			if (x<X){
			  return angle = 90;
			}
			else if (x==X){
				return angle = theta();
			}
			else{
				return angle = -90;
			}
		}
		// straight up and down x axis
		if (y-Y < 0) {
			if (x-X < 0){
				return angle = angle - 180;
			}
			else if (x-X > 0) {
				return angle = angle + 180;
			}
			else {
				return angle = -180;
			}
		}
		return angle;
	}

	public void forward(double distance, int speed){
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

		A_ang = A_ang + distance * (Lwheel_amt_per_cm) * 1.42857142857;
		B_ang = B_ang + distance * (Rwheel_amt_per_cm) * 1.42857142857;

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

	public void spotTurn_gyro(float angturn){		/**
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
					Motor.A.forward();
					Motor.B.backward();
				} else {
					Motor.A.backward();
					Motor.B.forward();

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
		 * @param a speed of the left wheel, in degrees per second
		 * @param b speed of the left wheel, in degrees per second
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


		for (int i = 0; i < 10; i++)
			sum += this.ping();

		return (float) sum / 10;
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

	public int count_house(){
		/**
		 * This function returns the current house number we are at
		 * 1, 2, 3 will be returned if we are in front of a house
		 * 0 will be returned if we are not in front of a house
		 *
		 */
		if(this.avg_ping() < (100)){
			// no house is in sight
			if (house_buffer_count > 50){
				if (house_edge == 0){
				// The robot didnt see a house in the last iteration, now there is house
				house_count += 1;
				house_edge = 1;
				}
			}else{
				// Ultrasonic sees object, but have not enough to trigger a detection
				house_buffer_count += 1;
				return 0;
			}
			
			
			return house_count;
		} else {
			
			if (house_edge == 1){
				// The robot was at a house in the last iteration, not anymore
				house_edge = 0;
			}
			house_buffer_count = 0;
			// return zero, no house
			return 0;
		}
	}

	public void drop_pizza(int house){
		if (house > 5){
			spotTurn_gyro((theta()+90));
		} else if (house <=5){
			spotTurn_gyro((theta()-90));
		}

		Motor.D.rotateTo(-25);
		Delay.msDelay(2000);
		//forward(10,100);
		//forward(10, 100);
		Motor.D.rotateTo(90);
		
	}

}
