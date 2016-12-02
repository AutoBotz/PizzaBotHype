package PizzaBotPkg;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.utility.Delay;

//import statements

/**
* @author      Andrej Janda, Ethan Waldie, Michael Ding, Yunsung Oh
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

	public int[] obtain_selection(){
		int[] selection = {1,1,1};
		int pizza_option = 1;
		System.out.println("Pizza option is " + pizza_option);
		System.out.println("1 for left side and 2 for right side");
		int stage = 1;
		while (stage == 1) {
			if (Button.ENTER.isDown()) {
				Delay.msDelay(100);
				stage = 2;
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
		selection[1] = pizza_option;

		int oval_desired = 1;
		System.out.println("Oval option is " + oval_desired);
		System.out.println("1 for Red, 2 for Blue, and 3 for Green");
		while (stage == 2) {
			if (Button.ENTER.isDown()) {
				Delay.msDelay(100);
				stage = 3;
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
		selection[2] = oval_desired;

		int house_desired = 1;
		System.out.println("House option is " + house_desired);
		System.out.println("1 for left first, 2 for left second, and 3 for left third");
		System.out.println("6 for right first, 7 for right second, and 8 for right third");
		while (stage == 3) {
			if (Button.ENTER.isDown()) {
				Delay.msDelay(100);
				stage = 4;
			} else if (Button.UP.isDown()) {
				Delay.msDelay(50);
				if (house_desired == 8) {
					house_desired = 1;
				} else if (house_desired == 3) {
					house_desired = 6;
				} else {
					house_desired += 1;
				}
				System.out.println("House option is " + house_desired);
			} else if (Button.DOWN.isDown()) {
				Delay.msDelay(50);
				if (house_desired == 1) {
					house_desired = 8;
				} else if (house_desired == 6) {
					house_desired = 3;
				} else {
					house_desired -= 1;
				}
				System.out.println("House option is " + house_desired);
			}
		}
		selection[3] = house_desired;

		return selection;
	}

}
