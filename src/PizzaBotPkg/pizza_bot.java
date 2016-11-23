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

	public static drive_control robot = new drive_control();
	public static user_interface UI = new user_interface();
	
	public static void main(String[] args) {
		robot.gyro_init(1);
		robot.sonic_init(2);
		robot.init_pos(0.0, 0.0);
		robot.set_dims(4, 4, 12);
	
	    move_to_point(15,15, robot);
	    move_to_point(15,-15, robot);
	    move_to_point(0, 0, robot);
	    
	    robot.spotTurn_gyro(0);
	    robot.flt();
	    
	    while(true){
	    	
	    	// Exit code
	    	if (Button.ESCAPE.isDown()) {
	    	Motor.A.flt();
	    	Motor.B.flt();
	    	Delay.msDelay(1000);
    
	    	if (Button.ESCAPE.isDown()){break;}
	    	}
	    	// End Exit code
	    	
	    	while(robot.ping() > 15){
	    		robot.forward(5, 5);
	    	}
	    	
	    }
	}
	
	public static void move_to_point(int x, int y, drive_control robot) {
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
		
		UI.println("Current Pos (" + (int)robot.X + " , " + (int)robot.Y +" , " +(int)robot.theta() +")");

		double delta_angle = (180*Math.atan((y-robot.Y)/(x-robot.X))/3.14159);
		double distance = Math.sqrt((y-robot.Y)*(y-robot.Y) + (x-robot.X)*(x-robot.X));
		

		// driving reverse
		if ((y-robot.Y)<0){
			if ((x-robot.X)<0){
				delta_angle = -90-delta_angle;
			}
			else
				delta_angle = 90 - delta_angle;
		}

		
		robot.spotTurn_gyro((int)delta_angle);
		
		System.out.println("going foreward");
		
		robot.forward((int)distance, 100);
		
	}
	
	public static int obstacle_encounter(float[] distance_array){
		/**
		 * This function accepts a array of 9 distance measurements and return the
		 * direction to turn that will avoid obstacle
		 *
		 * @param distance_array array of 9 distance measurements
		 */
		
		// Distance of robot to object distance_array[4]
		
		float curr_max = distance_array[4];
		int max_direction = 4;
		
		for(int i = 0; i < 9; i++){
			if (distance_array[i] < 250 && distance_array[i] > curr_max){
				curr_max = distance_array[i];
				max_direction = i;
			}
		}
		return (int)(max_direction*22.5 - 90);
	}


}

