package PizzaBotPkg;
//import lejos.utility.Stopwatch;
//import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
//import lejos.hardware.motor.Motor;
//import lejos.hardware.port.SensorPort;
//import lejos.hardware.sensor.EV3ColorSensor;
//import lejos.hardware.sensor.EV3GyroSensor;
//import lejos.utility.Delay;

//import statements

/**
* @author      Andrej Janda, Ethan Waldie, Michael Ding
* @version     0.1
* @since       0.0
*/


public class user_interface {
	public void clear_screen(){
		LCD.clear();
	}

	public void println(String x){
		System.out.println(x);
	}

	public void print(String x){
		System.out.print(x);
	}

	public int input_pizza(){
		int pizza_option = 1;
		System.out.println("Pizza option is " + pizza_option);
		System.out.println("1 for left side and 2 for right side");
		while (true) {
			if (Button.ENTER.isDown()) {
				Delay.msDelay(100);
				return pizza_option;
			} else if (Button.UP.isDown()) {
				Delay.msDelay(50);
				if (pizza_option == 1) {
					pizza_option = 2;
				} else {
					pizza_option = 1;
				}
				System.out.println("Pizza option is " + pizza_option);
			} else if (Button.DOWN.isDown()) {
				Delay.msDelay(50);
				if (pizza_option == 1) {
					pizza_option = 2;
				} else {
					pizza_option = 1;
				}
				System.out.println("Pizza option is " + pizza_option);
			}
		}
	}

	public int input_oval(){
		int oval_desired = 1;
		System.out.println("Oval option is " + oval_desired);
		System.out.println("1 for Red, 2 for Blue, and 3 for Green");
		while (true) {
			if (Button.ENTER.isDown()) {
				Delay.msDelay(100);
				return oval_desired;
			} else if (Button.UP.isDown()) {
				Delay.msDelay(50);
				if (oval_desired == 3) {
					oval_desired = 1;
				} else {
					oval_desired += 1;
				}
				System.out.println("Oval option is " + oval_desired);
			} else if (Button.DOWN.isDown()) {
				Delay.msDelay(50);
				if (oval_desired == 1) {
					oval_desired = 3;
				} else {
					oval_desired -= 1;
				}
				System.out.println("Oval option is " + oval_desired);
			}
		}
	}

	public int input_roadside(){
		int side_of_road = 1;
		System.out.println("Road side option is " + side_of_road);
		System.out.println("1 for left side and 2 for right side");
		while (true) {
			if (Button.ENTER.isDown()) {
				Delay.msDelay(100);
				return side_of_road;
			} else if (Button.UP.isDown()) {
				Delay.msDelay(50);
				if (side_of_road == 1) {
					side_of_road = 2;
				} else {
					side_of_road = 1;
				}
				System.out.println("Road side option is " + side_of_road);
			} else if (Button.DOWN.isDown()) {
				Delay.msDelay(50);
				if (side_of_road == 1) {
					side_of_road = 2;
				} else {
					side_of_road = 1;
				}
				System.out.println("Road side option is " + side_of_road);
			}
		}
	}

	public int input_house(){
		int house_desired = 1;
		System.out.println("House option is " + house_desired);
		System.out.println("1 for the first, 2 for the middle, and 3 for the last");
		while (true) {
			if (Button.ENTER.isDown()) {
				Delay.msDelay(100);
				return house_desired;
			} else if (Button.UP.isDown()) {
				Delay.msDelay(50);
				if (house_desired == 3) {
					house_desired = 1;
				} else {
					house_desired += 1;
				}
				System.out.println("House option is " + house_desired);
			} else if (Button.DOWN.isDown()) {
				Delay.msDelay(50);
				if (house_desired == 1) {
					house_desired = 3;
				} else {
					house_desired -= 1;
				}
				System.out.println("House option is " + house_desired);
			}
		}
	}
}
