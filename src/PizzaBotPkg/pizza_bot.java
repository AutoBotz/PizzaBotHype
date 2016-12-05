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
	
	public static final double[] red_road = {-39.0,  210.0,   -102.0,   345.0,    60};
	
	public static final double[] green_road = {40,   210.0,   111.0,   345.0,    -62.3};
	
	public static final double[] blue_road = {0,   210.0,    0,    360.0,    0};
	
	public static int goal_pizza = 0;
	public static double[] goal_road = new double[4];
	public static double goal_road_dir = 0.0;

	
	// Mission stage keep track of what part of the misson we are on
	// So the while loop can be operated at high frequency and we can record location or whatever
	// Mission stages:
	//
	//       0              1                 2                 3                  4  
	// Initialization    Take Pizza       Find Street        Find House         Return
	
	public static int mission_stage = 1;


	public static int oval_desired = 0;
	public static int house_desired = 3;
	public static int side_of_road = 0;
	public static int pizza_option = 0;


	public static void main(String[] args) {
		UI.println("START!");
		
		select_goal();
		robot.gyro_init(1);
		UI.println("angle " + robot.theta());
		robot.sonic_init(2);
		

		///robot.init_pos(goal_road[0], goal_road[1]);
		robot.set_dims(5.5, 5.5, 12);
		Motor.D.rotateTo(0);


	    while(true){

	    	// Exit code
	    	if (Button.ESCAPE.isDown()) {
	    	Motor.A.flt();
	    	Motor.B.flt();
	    	Delay.msDelay(1000);
	       	if (Button.ESCAPE.isDown()){break;}
	    	}

	    	//UI.println(" "+ robot.avg_ping());
	    	
	    	if (mission_stage == 5){
	    		// test house counting algo
	    		robot.set_speed(100, 100);
	    		Motor.A.forward();
	    		Motor.B.forward();
	    		System.out.print(robot.count_house()+"    ");
	    		System.out.println(robot.ping());
	    	}
	    	
	    	if (mission_stage == 4){
	    		robot.move_to_Point_PID_SONIC(0, -10, 200);
	    		robot.spotTurn_gyro(0);
	    		UI.clear_screen();
	    		UI.print("YOU ARE WINNER !!");
	    		UI.print("YOU ARE WINNER !!");
	    		UI.print("YOU ARE WINNER !!");
	    		mission_stage = 1337;
	    	}
	    	
	    	if (mission_stage == 3){
	    		
	    		// left or right side of the street, rotate ultrasonic to align
	    		if (house_desired > 5){
		    		Motor.C.rotateTo(-90);
	    		} else if (house_desired <= 5){
		    		Motor.C.rotateTo(90);
	    		}
	    		Motor.C.flt();
	    		
	    		// Drive forward to avoid the debris at the start
	    		//robot.forward(15, 150);
	    		//System.out.println(""+robot.X+"  "+robot.Y+"  "+goal_road[2]+"  "+goal_road[3]);
	    		//Delay.msDelay(2000);
	    		robot.move_to_house(goal_road[2],goal_road[3],200,house_desired);
	    		Motor.C.rotateTo(0);
	    		robot.move_to_Point_PID_SONIC(goal_road[0], goal_road[1], 150);
	    		// robot.move_to_Point_PID_SONIC(goal_road[0], goal_road[1], 200);
	    		// break;
	    		mission_stage = 4;
	    	}
	    	if (mission_stage == 2){
	    		//robot.move_to_Point_PID_SONIC(0,200, 200);
			    //robot.spotTurn_gyro(0);
			    
			   robot.move_to_Point_PID_SONIC(goal_road[0], goal_road[1], 150);
	    	   //robot.move_to_Point_PID_SONIC(39, 210, 150);
			   // 	robot.spotTurn_gyro((float)goal_road_dir);
	    		mission_stage = 3;

	    	}
	    	
	    	if (mission_stage == 1){
	    		pickup_pizza();
	    		System.out.println("(" + robot.X + "," + robot.Y + ")");
	    		mission_stage = 2;
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
		// int[] goal = {1,1,3};
		int[] goal = poll_input();
		
		if (goal[0] == 1){
			goal_pizza = 1;
		}else if(goal[0] == 2){
			goal_pizza = 2;
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
			UI.println("Street Selection Error");
		}
		
		if (goal[2] != 0){
			house_desired  = goal[2];
		} else {
			UI.println("House Selection Error");
		}
		
	}
	

	
	public static void pickup_pizza(){
		Motor.D.setSpeed(50);
		if (goal_pizza == 1){
			Motor.D.rotateTo(-10);
			robot.forward(18 / 1.42857142857, 200);
			robot.spotTurn_gyro(-90);
			robot.forward(-60 / 1.42857142857, 200);
			Motor.D.rotateTo(90);
			robot.forward(10, 200);
		}
		
		if (goal_pizza == 2){
			Motor.D.rotateTo(-10);
			robot.forward(18 / 1.42857142857, 200);
			robot.spotTurn_gyro(90);
			robot.forward(-60 / 1.42857142857, 2000);
			Motor.D.rotateTo(90);
			robot.forward(10, 200);
		}
	}
	
	public static int[] poll_input(){
		int pizza_sel = 0;
		int road_sel = 0;
		int house_sel = 0;
		String[] pizza_array = {"unselected(0)", "left(1)", "right(2)"};
		String[] road_array = {"unselected(0)", "red(1)", "blue(2)", "green(3)"};
		String[] house_array ={"unselected(0)", "left_first(1)", "left_second(2)", "left_last(3)","4","5","right_first(6)", "right_second(7)", "right_last(8)"};
		
		
		System.out.println("Please provide pizza selection");
		System.out.println("Pizza: " + pizza_array[pizza_sel]);
		while (true) {
			if (Button.ENTER.isDown()) {
				Delay.msDelay(100);
				break;
			} else if (Button.UP.isDown()) {
				Delay.msDelay(100);
				pizza_sel += 1;
				pizza_sel = Math.abs(pizza_sel)%3;
				System.out.println("Pizza: " + pizza_array[pizza_sel]);
			} else if (Button.DOWN.isDown()) {
				Delay.msDelay(100);
				pizza_sel -= 1;
				pizza_sel = Math.abs(pizza_sel)%3;
				System.out.println("Pizza: " + pizza_array[pizza_sel]);
			} else if (Button.LEFT.isDown()) {
				Delay.msDelay(100);
				pizza_sel = 0;
				System.out.println("Pizza: " + pizza_array[pizza_sel]);
			} 
		}
		
		System.out.println("Please provide street selection");
		System.out.println("Street: " + road_array[road_sel]);
		while (true) {
			if (Button.ENTER.isDown()) {
				Delay.msDelay(100);
				break;
			} else if (Button.UP.isDown()) {
				Delay.msDelay(100);
				road_sel += 1;
				road_sel = Math.abs(road_sel)%4;
				System.out.println("Street: " + road_array[road_sel]);
			} else if (Button.DOWN.isDown()) {
				Delay.msDelay(100);
				road_sel -= 1;
				road_sel = Math.abs(road_sel)%4;
				System.out.println("Street: " + road_array[road_sel]);
			} else if (Button.LEFT.isDown()) {
				Delay.msDelay(100);
				road_sel = 0;
				System.out.println("Street: " + road_array[road_sel]);
			} 
		}
		
		System.out.println("Please provide house selection");
		System.out.println("House: " + house_array[house_sel]);
		while (true) {
			if (Button.ENTER.isDown()) {
				Delay.msDelay(100);
				break;
			} else if (Button.UP.isDown()) {
				Delay.msDelay(100);
				house_sel += 1;
				house_sel = Math.abs(house_sel)%8;
				System.out.println("Street: " + house_array[house_sel]);
			} else if (Button.DOWN.isDown()) {
				Delay.msDelay(100);
				house_sel -= 1;
				house_sel = Math.abs(house_sel)%8;
				System.out.println("Street: " + house_array[house_sel]);
			} else if (Button.LEFT.isDown()) {
				Delay.msDelay(100);
				house_sel = 0;
				System.out.println("Street: " + house_array[house_sel]);
			} 
		}
		int [] return_val = {pizza_sel,road_sel,house_sel};
		return return_val;
		
	}
}
