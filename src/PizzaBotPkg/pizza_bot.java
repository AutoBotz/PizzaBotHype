package PizzaBotPkg;
import lejos.utility.Stopwatch;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
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

public class pizza_bot {

	public static void main(String[] args) {
		drive_control robot = new drive_control();
		robot.set_dims(4, 4, 12);
		robot.gyro_cal();	// calibrate gyro
	
		
		
	    move_to_point(15,15, robot);
	    move_to_point(15,-15, robot);
	    move_to_point(0, 0, robot);
	    
	    robot.spotTurn_gyro(0);
	    
	    
		
	}
	
	public static void move_to_point(int x, int y, drive_control robot) {
		
		System.out.println("Current Pos (" + (int)robot.X + " , " + (int)robot.Y +" , " +(int)robot.theta() +")");

		
		
		double new_angle = (180*Math.atan((y-robot.Y)/(x-robot.X))/3.14159);
		double distance = Math.sqrt((y-robot.Y)*(y-robot.Y) + (x-robot.X)*(x-robot.X));
		
		System.out.println(new_angle + " "+ distance);

		// driving reverse
		if ((y-robot.Y)<0){
			if ((x-robot.X)<0){
				new_angle = -90-new_angle-robot.theta();
			}
			else
				new_angle = 90 - new_angle;
		}

		
		robot.spotTurn_gyro((int)new_angle);
		
		System.out.println("going foreward");
		
		robot.forward((int)distance, 100);
		
	}
}

