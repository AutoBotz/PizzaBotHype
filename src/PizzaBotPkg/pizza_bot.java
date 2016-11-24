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
	public static float house_distance = 10; // WE NEED TO MEASURE THIS
	public static int house_count = 0;
	public static int house_edge = 0;
<<<<<<< HEAD
	public static int house_desired = 0;
=======
>>>>>>> origin/master

	// Mission stage keep track of what part of the misson we are on
	// So the while loop can be operated at high frequency and we can record location or whatever
	// Mission stages:
	//
	//       0              1                 2                 3                  4                5
	// Initialization    Take Pizza     Obstacle Avoid       Find Street        Find House         Return

<<<<<<< HEAD
	public static int mission_stage = 0;

	public static int oval_desired = 0;
=======
	public static int mission_stage = 2;
>>>>>>> origin/master

	public static int house_desired = 3;
	public static int side_of_road = 0;
	public static int pizza_option = 0;


	public static void main(String[] args) {
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

<<<<<<< HEAD
				if (mission_stage == 0){
	    		UI.println("Start input parameters");
					pizza_option = UI.input_pizza();
					oval_desired = UI.input_oval();
					side_of_road = UI.input_roadside();
					house_desired = UI.input_house();
	    		mission_stage = 1;
	    	}

=======
>>>>>>> origin/master
	    	//UI.println(" "+ robot.avg_ping());

	    	if (mission_stage == 1){
	    		pickup_pizza();
	    		mission_stage = 2;
	    	}

	    	if (mission_stage == 2){
		    	if (Button.ENTER.isDown()) {
		    			robot.gyro_init(1);
					    move_to_point(0,200, robot);
					    UI.println("(" + (int)robot.X + " , " + (int)robot.Y +" , " +(int)robot.theta() +")");
					    Delay.msDelay(2000);
					    move_to_point(35,200, robot);
					    UI.println("(" + (int)robot.X + " , " + (int)robot.Y +" , " +(int)robot.theta() +")");
					    Delay.msDelay(2000);
					    move_to_point(-35,200, robot);
					    UI.println("(" + (int)robot.X + " , " + (int)robot.Y +" , " +(int)robot.theta() +")");
					    Delay.msDelay(2000);

					    robot.spotTurn_gyro(0);
				    }
	    	}

	    	if (mission_stage == 3){

	    		// left or right side of the street, rotate ultrasonic to align
	    		if (side_of_road == 1){
		    		Motor.C.rotateTo(-90);
	    		} if (side_of_road == 2){
		    		Motor.C.rotateTo(90);
	    		}

	    		// Drive forward to avoid the debris at the start
	    		robot.forward(10, 150);
	    	}

	    	if (mission_stage == 4){
	    		while (count_house() < house_desired){
	    			robot.forward(2, 150);
	    		}

	    		drop_pizza();
	    	}

	    	}
	}

	public static void move_to_point(int x, int y, drive_control robot) {
		//UI.println("(" + (int)robot.X + " , " + (int)robot.Y +" , " +(int)robot.theta() +")");

		double delta_angle = desired_Orientation(x,y,robot);
		double distance = Math.sqrt((y-robot.Y)*(y-robot.Y) + (x-robot.X)*(x-robot.X));

		UI.println("" + delta_angle);

		robot.spotTurn_gyro((int)delta_angle);

		// Check for obstical
		for (int i = 0; i < (int)distance; i++) {
			float obj_dist = robot.ping();
			if (obj_dist < 10) {
				object_avoid();
				move_to_point(x,y,robot);
				break;
			} else {
				robot.forward(1, 100);
			}
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

	public static int count_house(){
		/**
		 * This function returns the current house number we are at
		 * 1, 2, 3 will be returned if we are in front of a house
		 * 0 will be returned if we are not in front of a house
		 *
		 */

		if(robot.avg_ping() > (house_distance + 3)){
			// no house is in sight
			if (house_edge == 1){
				// The robot was at a house in the last iteration, not anymore
				house_edge = 0;
			}

			// return zero, no house
			return 0;
		} else {
			// ultrasonic see object that is within distance of a house, so there is probably one
			if (house_edge == 0){
				// The robot didnt see a house in the last iteration, now there is house
				house_count += 1;
				house_edge = 1;
			}
			return house_count;
		}
	}

	public static void pickup_pizza(){
		robot.forward(1.1, 150);
		if (pizza_option == 1){
			robot.spotTurn(90, 150);

		} else if (pizza_option == 2){
			robot.spotTurn(270, 150);
		}

		Motor.D.rotateTo(-10);
		robot.forward(-5,150);
		Motor.D.rotateTo(90);
	}

	public static void drop_pizza(){
		if (side_of_road == 1){
			robot.spotTurn(90, 150);
		} else if (side_of_road == 2){
			robot.spotTurn(270, 150);
		}

		Motor.D.rotateTo(-25);
		Delay.msDelay(2000);
		Motor.D.rotateTo(90);

	}
}
