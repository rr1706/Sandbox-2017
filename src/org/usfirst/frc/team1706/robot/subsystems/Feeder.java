package org.usfirst.frc.team1706.robot.subsystems;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Feeder {
	static Talon feederBelt = new Talon(4);
	static Talon feederWheels = new Talon(5);
	
	static boolean started = false;
	static boolean timeStarted = false;
	static double time1;
	static double time2;

	public static void start() {
		feederBelt.set(-1.0);
		
		if (!timeStarted) {
			time1 = Time.get();
			timeStarted = true;
		}
		
		time2 = Time.get();
		
		if (Math.abs(time2 - time1) > 1) {
			started = true;
		
		}
		
		System.out.println(started);
		
		if (!LogitechController.RTrig()) {
			if (!started) {
				feederWheels.set(-1);
			} else {
				feederWheels.set(-0.6);
			}
			Light.setFeed();
		}		
	}

	public static void stop() {
		feederBelt.set(0);
		feederWheels.set(0);
		started = false;
	
	}
	
	public static void reverse() {
		feederWheels.set(1);
	}
	
	public static void lightStop() {
		feederBelt.set(0);
		feederWheels.set(0);
		started = false;
		
	}
}

