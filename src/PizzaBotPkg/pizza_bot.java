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
	
	// Constants
	// Record the absolute positions of various destinations here to call upon
	

	//public static final double[] pizza1_loc = {15.0, 60.0};
	//public static final double[] pizza2_loc = {15.0, -60.0};
	
	//public static final double[] pizza1_loc = {13.0, 58.0};
	//public static final double[] pizza2_loc = {13.0, -58.0};
	
	public static final double[] red_road = {39.0,  210.0,   45.0,   345.0,    60};
	
	public static final double[] green_road = {-40,   210.0,   -45.0,   345.0,    -62.3};
	
	public static final double[] blue_road = {0,   210.0,    0,    360.0,    0};
	
	public static double[] goal_pizza = new double[2];
	public static double[] goal_road = new double[4];
	public static double goal_road_dir = 0.0;

	
	// Mission stage keep track of what part of the misson we are on
	// So the while loop can be operated at high frequency and we can record location or whatever
	// Mission stages:
	//
	//       0              1                 2                 3                  4  
	// Initialization    Take Pizza       Find Street        Find House         Return
	
	public static int mission_stage = 3;


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
		Motor.D.rotateTo(0);
		
		robot.X = green_road[0];
		robot.Y = green_road[1];


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
		    		robot.move_to_Point_PID_SONIC(0,200, 200);
				    //robot.spotTurn_gyro(0);
				    
				    //robot.move_to_Point_PID_SONIC(goal_road[0], goal_road[1], 150);
				    //	robot.spotTurn_gyro((float)goal_road_dir);
			    }
	    	}
	    	
	    	if (mission_stage == 3){
	    		
	    		// left or right side of the street, rotate ultrasonic to align
	    		/*if (house_desired > 5){
		    		Motor.C.rotateTo(-90);
	    		} else if (house_desired <= 5){
		    		Motor.C.rotateTo(90);
	    		}*/
	    		
	    		// Drive forward to avoid the debris at the start
	    		//robot.forward(15, 150);
	    		//System.out.println(""+robot.X+"  "+robot.Y+"  "+goal_road[2]+"  "+goal_road[3]);
	    		//Delay.msDelay(2000);
	    		robot.move_to_house(goal_road[2],goal_road[3],150,house_desired);
	    		
	    	}
	    	
	    	if (mission_stage == 4){
	    		robot.move_to_Point_PID_SONIC(0, 0, 200);
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
		 * [1,x,x] select pizza 1
		 * [2,x,x] select pizza 2
		 * 
		 * [x,1,x] select red road
		 * [x,2,x] select blue road
		 * [x,3,x] select green road
		 * 
		 * [x,x,1] select left first
		 * [x,x,2] select left second
		 * [x,x,3] select left third
		 * [x,x,6] select right first
		 * [x,x,7] select right second
		 * [x,x,8] select right third
		 *  
		 */
		// int[] goal = UI.obtain_selection();
		int[] goal = {1,3};
		
		if (goal[0] == 1){
			//goal_pizza = pizza1_loc;
		}else if(goal[0] == 2){
			//goal_pizza = pizza2_loc;
		} else {
			UI.println("Pizza Selection Error");
		}
	
		if (goal[1] == 1){
			goal_road[0] = red_road[0];
			goal_road[1] = red_road[1];
			goal_road[2] = red_road[2];
			goal_road[3] = red_road[3];
			goal_road_dir = red_road[4];
		}else if(goal[1] == 2){
			goal_road[0] = blue_road[0];
			goal_road[1] = blue_road[1];
			goal_road[2] = blue_road[2];
			goal_road[3] = blue_road[3];
			goal_road_dir = blue_road[4];
		}else if(goal[1] == 3){
			goal_road[0] = green_road[0];
			goal_road[1] = green_road[1];
			goal_road[2] = green_road[2];
			goal_road[3] = green_road[3];
			goal_road_dir = green_road[4];
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

	
	public static void pickup_pizza(){
		Motor.D.rotateTo(-10);
		robot.forward(15.5, 100);
		robot.spotTurn_gyro(90);
		robot.forward(-60, 100);
		//robot.reverse_to_Point_PID(goal_pizza[0], goal_pizza[1],100);
		Motor.D.rotateTo(90);
		robot.forward(15, 100);
	}
	
	
}
