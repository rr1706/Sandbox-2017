package org.usfirst.frc.team1706.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;

//Plug in second
public class LogitechController {
	private static Joystick stick = new Joystick(1);

	public static boolean X() {
		return stick.getRawButton(1);
	}

	public static boolean A() {
		return stick.getRawButton(2);
	}

	public static boolean B() {
		return stick.getRawButton(3);
	}

	public static boolean Y() {
		return stick.getRawButton(4);
	}

	public static boolean LB() {
		return stick.getRawButton(5);
	}

	public static boolean RB() {
		return stick.getRawButton(6);
	}

	public static boolean LTrig() {
		return stick.getRawButton(7);
	}

	public static boolean RTrig() {
		return stick.getRawButton(8);
	}

	public static boolean Back() {
		return stick.getRawButton(9);
	}

	public static boolean Start() {
		return stick.getRawButton(10);
	}

	public static boolean LStickButton() {
		return stick.getRawButton(11);
	}

	public static boolean RStickButton() {
		return stick.getRawButton(12);
	}

	public static double LStickX() {
		return stick.getRawAxis(0);
	}

	public static double LStickY() {
		return stick.getRawAxis(1);
	}

	public static double RStickX() {
		return stick.getRawAxis(2);
	}

	public static double RStickY() {
		// System.out.println(stick.getRawAxis(3));
		if (Math.abs(stick.getRawAxis(3)) >= 0.1) {
			// System.out.println("1");
			return stick.getRawAxis(3);
		} else {
			// System.out.println("2");
			return 0.0;

		}
	}

	public static int DPad() {
		return stick.getPOV();
	}

}
