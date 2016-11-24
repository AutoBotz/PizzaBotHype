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
	public static double pi = Math.PI;

	public static void main(String[] args) {
		robot.gyro_init(1);
		robot.sonic_init(2);

		robot.init_pos(0.0, 0.0);
		robot.set_dims(4, 4, 12);


	    while(true){

	    	// Exit code
	    	if (Button.ESCAPE.isDown()) {
	    	Motor.A.flt();
	    	Motor.B.flt();
	    	Delay.msDelay(1000);
	       	if (Button.ESCAPE.isDown()){break;}
	    	}

	    	// End Exit code
	    	if (Button.ENTER.isDown()) {
				    move_to_point(0,150, robot);
						UI.println("Current Pos (" + (int)robot.X + " , " + (int)robot.Y +" , " +(int)robot.theta() +")");

				    robot.spotTurn_gyro(0);
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
		//UI.println("(" + (int)robot.X + " , " + (int)robot.Y +" , " +(int)robot.theta() +")");

		double delta_angle = desired_Orientation(x,y,robot);
		double distance = Math.sqrt((y-robot.Y)*(y-robot.Y) + (x-robot.X)*(x-robot.X));

		UI.println("" + delta_angle);

		robot.spotTurn_gyro((int)delta_angle);

		// Check for obstical
		for (int i = 0; i < (int)distance; i++) {
			float obj_dist = robot.ping();
			if (obj_dist < 10) {
				this.object_avoid();
				move_to_point(x,y,robot);
				break;
			} else {
				robot.forward(1, 100);
			}
		}

	}

	public static int object_avoid(){
		// Avoid object by turning right and
		// traveling 20 cm
		robot.spotTurn_gyro(robot.theta() + 90);
		robot.forward(20, 150);
	}

	public static double desired_Orientation(int x, int y, drive_control robot){
		// Set angle taking into account boundary cases

		double angle = (180*Math.atan((y-robot.Y)/(x-robot.X))/pi);

		// straight up and down y axis
		if (x-robot.X==0){
			   if (y-robot.Y >=0){
			    return angle = 0;
			   }
			   else{
			    return angle = -180;
			   }

	  // straigh up and down x axis
		} else if (y-robot.Y==0) {
			   if (x-robot.X >=0){
				    return angle = 90;
				   }
				   else{
				    return angle = -90;
			}

		// angles behind robot
		} else {
			if ((y-robot.Y)<0){
				if ((x-robot.X)<0){
					return angle = -90 - angle;
				}
				else
					return angle =  90 - angle;
			}
			else {return angle}
		}
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
