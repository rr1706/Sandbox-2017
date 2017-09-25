package org.usfirst.frc.team1706.robot.subsystems;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team1706.robot.subsystems.PowerPanel;
import org.usfirst.frc.team1706.robot.subsystems.LogitechController;

public class Climber {

	static Talon climber1 = new Talon(7);
	static Talon climber2 = new Talon(8);
		
	public static void run(double power) {
		climber1.set(-power);
		climber2.set(-power);
		if (power != 0) {
			Light.setClimb();
		}
	}
	
	public static void stop() {
		climber1.set(0.0);
		climber2.set(0.0);
	}
}
