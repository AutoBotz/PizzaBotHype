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
	
	// Constants
	// Record the absolute positions of various destinations here to call upon
	
	
	public static final float[] pizza1_loc = {Pizza_X, Pizza_Y};
	public static final float[] pizza2_loc = {Pizza_X, Pizza_Y};
	
	public static final float[] red_road = {red_start_X, red_start_Y,red_direction_angle};
	
	public static final float[] green_road = {green_start_X, green_start_Y,green_direction_angle};
	
	public static final float[] blue_road = {blue_start_X, blue_start_Y,blue_direction_angle};
	
	public static float[] goal_pizza;
	public static float[] goal_road;
	public static float goal_road_dir;

	
	// Mission stage keep track of what part of the misson we are on
	// So the while loop can be operated at high frequency and we can record location or whatever
	// Mission stages:
	//
	//       0              1                 2                 3                  4                5
	// Initialization    Take Pizza     Obstacle Avoid       Find Street        Find House         Return
	
	public static int mission_stage = 2;


	public static int oval_desired = 0;
	public static int house_desired = 3;
	public static int side_of_road = 0;
	public static int pizza_option = 0;


	public static void main(String[] args) {
		UI.println("START!");
		robot.gyro_init(1);
		UI.println("angle " + robot.theta());
		robot.sonic_init(2);
		
		select_goal();
		

		robot.init_pos(0.0, 0.0);
		robot.set_dims(5.5, 5.5, 12);


	    while(true){

	    	// Exit code
	    	if (Button.ESCAPE.isDown()) {
	    	Motor.A.flt();
	    	Motor.B.flt();
	    	Delay.msDelay(1000);
	       	if (Button.ESCAPE.isDown()){break;}
	    	}

	    	//UI.println(" "+ robot.avg_ping());
	    	
	    	if (mission_stage == 1){
	    		pickup_pizza();
	    		mission_stage = 2;
	    	}

	    	if (mission_stage == 2){
		    	if (Button.ENTER.isDown()) {
		    		robot.move_to_Point_PID_SONIC(0,30, 150);
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


	public static void select_goal(){
		/**
		 * This function sets the goal variables depending on input from the operator
		 * The selection will be returned in a 2 in array from function UI.obtain_selection();
		 * 
		 * obtain selection must return 2 in array with following rules
		 * 
		 * [1,x] select pizza 1
		 * [2,x] select pizza 2
		 * 
		 * [x,1] select red road
		 * [x,2] select blue road
		 * [x,3] select green road
		 *  
		 */
		int[] goal = UI.obtain_selection();
		
		if (goal[0] == 1){
			goal_pizza = pizza1_loc;
		}else if(goal[0] == 2){
			goal_pizza = pizza2_loc;
		} else {
			UI.println("Pizza Selection Error");
		}
	
		if (goal[1] == 1){
			goal_road[0] = red_road[0];
			goal_road[1] = red_road[1];
			goal_road_dir = red_road[2];
		}else if(goal[1] == 2){
			goal_road[0] = blue_road[0];
			goal_road[1] = blue_road[1];
			goal_road_dir = blue_road[2];
		}else if(goal[1] == 3){
			goal_road[0] = green_road[0];
			goal_road[1] = green_road[1];
			goal_road_dir = green_road[2];
		} else {
			UI.println("Road Selection Error");
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
