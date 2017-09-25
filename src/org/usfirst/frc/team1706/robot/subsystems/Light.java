package org.usfirst.frc.team1706.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Light {

	static int LEDState; 
	static int side;
	
	public static void setDisabled() {
		LEDState = 0;
	}
	
	public static void setAuto() {
		LEDState = 1;
	}

	public static void setTeleop() {
		LEDState = 2;
	}

	public static void setShoot() {
		LEDState = 3;
	}

	public static void setFeed() {
		LEDState = 4;
	}

	public static void setCollision() {
		LEDState = 5;
	}

	public static void setClimb() {
		LEDState = 6;
	}

	public static void setIntake() {
		LEDState = 7;
	}
	
	public static void setVision() {
		LEDState = 8;
	}
	
	public static void setSide(int x) {
		if (x == 1) {
			side = 0; //red
		} else {
			side = 10; //blue
		}
	}
	
	public static int getState(){
		return (LEDState + side) * 10;
	}
}
