package org.usfirst.frc.team1706.robot.subsystems;

import org.usfirst.frc.team1706.robot.Ds;

import edu.wpi.first.wpilibj.Talon;

public class Intake {

	static Talon intake = new Talon(6);
	static double command = 0;

	public static void start() {
		command = 0.8;//-0.0489*Math.pow(Ds.getBatteryVoltage(), 2) + 1.2415*Ds.getBatteryVoltage() - 7.1447;
//		if (command < 0.45) {
//			command = 0;
//		}
		intake.set(command);
		Light.setIntake();
	}
	
	public static void reverse() {
		intake.set(-1.0);
	}
	
	public static void stop() {
		intake.set(0.0);
	}
}
