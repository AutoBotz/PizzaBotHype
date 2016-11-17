package PizzaBotPkg;
import lejos.utility.Stopwatch;
import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor; 
import lejos.utility.Delay;


public class drive_control {
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