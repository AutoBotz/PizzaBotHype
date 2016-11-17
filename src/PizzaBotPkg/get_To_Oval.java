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


public class get_To_Oval {

	public static void main(String[] args) {

		drive = new drive_base;
		drive.set_dims(5.5, 5.5, 9.2);

		System.out.println(drive.x());
		System.out.println(drive.y());
		System.out.println(drive.theta());




	}





}
