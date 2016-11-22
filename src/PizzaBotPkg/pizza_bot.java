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
		user_interface UI = new user_interface();
		
		UI.clear_screen();
		
		
		robot.set_dims((float)5.5, (float)5.5, (float)9.2);
		robot.gyro_init();	// Initialize gyro
		
		System.out.println(robot.theta());

		robot.spotTurn_gyro(90);
		//robot.set_speed(10,10);
	}

}
