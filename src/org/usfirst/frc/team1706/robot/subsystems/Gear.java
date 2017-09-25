package org.usfirst.frc.team1706.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;

import org.usfirst.frc.team1706.robot.Robot;
import org.usfirst.frc.team1706.robot.subsystems.IMU;

public class Gear {
		
	static DriverStation ds = DriverStation.getInstance();
	
	static int state = 0;
	
	static Solenoid presser = new Solenoid(1);
	static DoubleSolenoid holder = new DoubleSolenoid(3, 2);
	
	static DigitalInput limitSwitch;
	
	static boolean previousButton = true;
	static boolean currentButton = true;
	
	static double prevTime = 0;
	
	static int autoCommand = 0;
	
	static double FWD = 0;
	
	static boolean firstRun = true;
	
	public static void run() {
		SmartDashboard.putNumber("Gear State", state);
		SmartDashboard.putBoolean("Have Gear", !limitSwitch.get());
		System.out.println(state + " | " + autoCommand);
		
		switch (state) {
		
			//Up without gear
			case 0: 
				holder.set(DoubleSolenoid.Value.kReverse);
				if (LogitechController.RB()) {
					state = 1;
					prevTime = Time.get();
				}
				
				if (firstRun) {
					presser.set(true);
					firstRun = false;
				}
				
				if (autoCommand == 1 && ds.isAutonomous()) {
					state = 5;
				}
			break;
			
			//Down
			case 1:
				holder.set(DoubleSolenoid.Value.kForward);
				presser.set(false);
				
				if (ds.isAutonomous()) {
					if (autoCommand == 0) {
						state = 0;
					}
				} else if (!LogitechController.RB()) {
					
					state = 0;
					
				} else if ((!limitSwitch.get() && Math.abs(prevTime - Time.get()) > 0.3) || LogitechController.LB()) {
					if (IMU.getVelocity() < 0.7) {
						state = 3;
						prevTime = Time.get();
					} else {
						state = 2;
					}
					
				}
				
				
			break;
			
			//Up waiting to go down
			case 2:
				holder.set(DoubleSolenoid.Value.kReverse);
				presser.set(true);
				
				currentButton = LogitechController.RB();
				if (currentButton && !previousButton) {
					state = 5;
					prevTime = Time.get();
				}
				previousButton = currentButton;
			break;
			
			//Start down
			case 5:
				
				holder.set(DoubleSolenoid.Value.kForward);
				
				if (Math.abs(prevTime - Time.get()) > 0.05) {
					state = 1;
					prevTime = Time.get();
				}
				
			break;
			
			//Press and Forward
			case 3:
//				holder.set(DoubleSolenoid.Value.kReverse);
				presser.set(true);
				
				
				FWD = -0.3;
				
				if (Math.abs(prevTime - Time.get()) > 0.2) {
					state = 4;
					prevTime = Time.get();
				}

			break;
			
			//Up and back
			case 4:
				holder.set(DoubleSolenoid.Value.kReverse);
				presser.set(true);
				
				FWD = 0.4;
				
				if (Math.abs(prevTime - Time.get()) > 0.5) {
					state = 2;
				}
			break;
		
		}
	}
	
	public static boolean getTrigger() {
		if (state == 3 || state == 4) {
			return  true;
		} else {
			return false;
		}
	}
	
	public static double getFWD() {
		return FWD;
	}
	
	public static void auto(double commands){
		
		autoCommand = (int) commands;
//		if (commands == 0) {
//			holder.set(DoubleSolenoid.Value.kReverse);
//			presser.set(true);
//		} else if (commands == 1) {
//			holder.set(DoubleSolenoid.Value.kForward);
//			presser.set(false);
//		}
	}
	
	public static void init() {
		limitSwitch = new DigitalInput(8);
	}
}
