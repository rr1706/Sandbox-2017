package org.usfirst.frc.team1706.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;

//FIXME Use the training version of this next season

//Plug in first
public class XboxController {
	private static Joystick stick = new Joystick(0);
	
	public static boolean A() {
		return stick.getRawButton(1);
	}

	public static boolean B() {
		return stick.getRawButton(2);
	}

	public static boolean X() {
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

	public static boolean Back() {
		return stick.getRawButton(7);
	}

	public static boolean Start() {
		return stick.getRawButton(8);
	}

	public static boolean LStickButton() {
		return stick.getRawButton(9);
	}

	public static boolean RStickButton() {
		return stick.getRawButton(10);
	}

	public static double LStickX() {
		return stick.getRawAxis(0);
	}

	public static double LStickY() {
		return stick.getRawAxis(1);
	}

	public static double LTrig() {
		return stick.getRawAxis(2);
	}

	public static double RTrig() {
		return stick.getRawAxis(3);
	}

	public static double RStickX() {
		return stick.getRawAxis(4);
	}

	public static double RStickY() {
		return stick.getRawAxis(5);
	}

	public static int DPad() {
		return stick.getPOV();
	}
	
	public static void rumble() {
		stick.setRumble(RumbleType.kRightRumble, 1);
		stick.setRumble(RumbleType.kLeftRumble, 1);
	}

	public static void stopRumble() {
		stick.setRumble(RumbleType.kRightRumble, 0);
		stick.setRumble(RumbleType.kLeftRumble, 0);
	}
}