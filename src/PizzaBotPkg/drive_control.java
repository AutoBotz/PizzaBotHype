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

	public static float pi = (float) Math.PI;


<<<<<<< HEAD:src/PizzaBotPkg/drive_base.java
    public float X = 0;
    public float Y = 0;

	// Sensor ports
	public EV3GyroSensor gyro = new EV3GyroSensor(SensorPort.S1);


=======
	  public float X = 0;
	  public float Y = 0;

	// Programmatics
	public static EV3GyroSensor gyro = new EV3GyroSensor(SensorPort.S1);
	public int gyro_sample_size = gyro.sampleSize();
	public float[] gyro_sample = new float[gyro_sample_size];

>>>>>>> origin/master:src/PizzaBotPkg/drive_control.java
	public void set_dims(float left_diameter, float right_diameter, float wheel_base){
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


	}


	public void forward(int distance, int speed){

	set_speed(speed, speed);

     double angle = (double)theta();

     double x = distance*Math.cos(angle);
     double y = distance*Math.sin(angle);

     X += x;
     Y += y;

		 double A_ang = Motor.A.getTachoCount();
		 double B_ang = Motor.B.getTachoCount();

		 A_ang = A_ang + distance * (360/17);
		 B_ang = B_ang + distance * (360/17);

		 Motor.A.rotateTo((int)(A_ang), true);
		 Motor.B.rotateTo((int)(B_ang));
	}

	public void spotTurn(int angturn, int speed){
		 set_speed(speed, speed);
		 int A_ang = Motor.A.getTachoCount();
		 int B_ang = Motor.B.getTachoCount();
		 int turnAmt = angturn ;


		 A_ang = A_ang - turnAmt;
		 B_ang = B_ang + turnAmt;

		 Motor.A.rotateTo(A_ang, true);
		 Motor.B.rotateTo(B_ang);
	}
<<<<<<< HEAD:src/PizzaBotPkg/drive_base.java
=======
	
>>>>>>> origin/master:src/PizzaBotPkg/drive_control.java
	public void spotTurn_gyro(int angturn){
			int K = 1;

			while (Math.abs(theta() - angturn) > 5) {
				float speed = K*(theta() - angturn);

				set_speed((int)speed, (int)speed);
				if (angturn > 0) {
					Motor.A.forward();
					Motor.B.backward();
				} else {
					Motor.A.backward();
					Motor.B.forward();
				}
			}

			stop();
		}
<<<<<<< HEAD:src/PizzaBotPkg/drive_base.java
	public void turn(int nominator, int denominator, int Speed, int direction) {
=======
	
	static void turn(int nominator, int denominator, int Speed, int direction) {
>>>>>>> origin/master:src/PizzaBotPkg/drive_control.java
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


	public void set_speed(int a, int b) {
		   Motor.A.setSpeed(a);
		   Motor.B.setSpeed(b);
			 Motor.A.forward();
			 Motor.B.forward();
	}

<<<<<<< HEAD:src/PizzaBotPkg/drive_base.java
	public void stop() {
			Motor.A.flt();
			Motor.B.flt();
	}

	public void gyro_cal() {
		
		gyro.reset();
		
		int sampleSize = gyro.sampleSize(); 
		float[] tiltsample = new float[sampleSize]; 
		float[] ratesample = new float[sampleSize]; 
		gyro.getAngleMode().fetchSample(tiltsample, 0); 
		
		System.out.println(tiltsample[0]);
		
	}

	public float theta() {
		
		float[] samples = new float[50];
		
		for (int i = 0; i < 50; i++) {
			int sampleSize = gyro.sampleSize(); 
			float[] tiltsample = new float[sampleSize]; 
			float[] ratesample = new float[sampleSize]; 
			gyro.getAngleMode().fetchSample(tiltsample, 0); 
			samples[i] = tiltsample[0];
		}
		
		
		
=======
	public void flt() {
			Motor.A.flt(true);
			Motor.B.flt();
	}

	public void stop() {
			Motor.A.stop(true);
			Motor.B.stop();
	}

	public void gyro_init(){
		gyro.getAngleMode(); 		// Set to purely angle mode
		gyro_sample_size = gyro.sampleSize(); //Modify gyro sample buffer to account of change of mode
		gyro_sample = new float[gyro_sample_size];
		
		this.gyro_cal(); // Call gyro calibration to recalibrate gyro
	}
	
	public void gyro_cal() {
		this.stop(); // Full stop, robot must be stationary for gyro calibration
		System.out.println("Hold for gyro calibration");
		Delay.msDelay(1000);
		gyro.reset(); 					// Reset the gyro
		
		// Wait for gyro to finish calibrating
		// will output NaN until calibration complete
		while (theta() == Float.NaN){
			Delay.msDelay(40);
		}

		System.out.println("Gyro calibration complete");
	}

	public float theta() {
		gyro.fetchSample(gyro_sample,0);
		return gyro_sample[0];
>>>>>>> origin/master:src/PizzaBotPkg/drive_control.java
	}


}
