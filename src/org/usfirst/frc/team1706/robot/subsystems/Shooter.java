package org.usfirst.frc.team1706.robot.subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
	static SendableChooser<Integer> debugShooter;
	static int shooterMode;

	static CANTalon masterShooter = new CANTalon(5);
	static CANTalon followerShooter = new CANTalon(4);
	static double targetSpeed = 0;
	static boolean active;
	
	static double prevTime;
	static boolean pulse = false;
	
	static boolean autonomous;

	public static void shoot(double distance) {
		debugShooter = new SendableChooser<Integer>();
		debugShooter.addDefault("Normal", 1);
		debugShooter.addObject("Debug", 2);
		SmartDashboard.putData("Shooter Mode Chooser", debugShooter);
		
		shooterMode = (int) debugShooter.getSelected();
		
		masterShooter.setF(SmartDashboard.getNumber("PIDF", 0.009));
		masterShooter.setP(SmartDashboard.getNumber("PIDP", 0.00165));
		masterShooter.setI(SmartDashboard.getNumber("PIDI", 0.0));
		masterShooter.setD(SmartDashboard.getNumber("PIDD", 0.0));
		
		if (shooterMode == 1) {
			masterShooter.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
			masterShooter.reverseSensor(true);
			
			followerShooter.changeControlMode(TalonControlMode.Follower);
			followerShooter.set(masterShooter.getDeviceID());
			
			if (distance != 9999 && distance != 0) {
				targetSpeed = 0.006 * Math.pow(distance, 3) - 1.3952 * Math.pow(distance, 2) + 119.5 * distance + 1958.7 + SmartDashboard.getNumber("Shooter Offset", 0);
			} else {
				targetSpeed = SmartDashboard.getNumber("RPM", 6000) + SmartDashboard.getNumber("Shooter Offset", 0);
			}
			
			if (LogitechController.Back()) {
				targetSpeed = 4500.0;
			}
			
			System.out.println(targetSpeed);
			
			masterShooter.changeControlMode(TalonControlMode.Speed);
			masterShooter.set(targetSpeed / 0.5625);
			
		} else if (shooterMode == 2) {
			if (LogitechController.X()) {
				masterShooter.changeControlMode(TalonControlMode.PercentVbus);
				masterShooter.set(-1.0);
				
				followerShooter.set(0.0);
				
			}
			
			if (LogitechController.Y()) {
				followerShooter.changeControlMode(TalonControlMode.PercentVbus);
				followerShooter.set(-1.0);
				
				masterShooter.set(0.0);
				
			}
			
			if (LogitechController.Start()) {
				masterShooter.changeControlMode(TalonControlMode.PercentVbus);
				followerShooter.changeControlMode(TalonControlMode.PercentVbus);
				
				masterShooter.set(0.0);
				followerShooter.set(0.0);
			}
		}
		
		double motorOutput = masterShooter.getOutputVoltage() / masterShooter.getBusVoltage();

		SmartDashboard.putNumber("Motor Output %", motorOutput * 100);
		SmartDashboard.putNumber("Encoder RPM", masterShooter.get() * 0.5625);
		SmartDashboard.putNumber("Shooter Temp", masterShooter.getTemperature()*9/5+32);
		
		//Use to test motor
//		masterShooter.changeControlMode(TalonControlMode.PercentVbus);
//		masterShooter.set(-0.15);
		
		if (!pulse && Time.get() - prevTime > 0.4) {
			pulse = !pulse;
			
			prevTime = Time.get();
			
		} else if (pulse && Time.get() - prevTime > 0.3) {
			pulse = !pulse;
			
			prevTime = Time.get();
			
		}
		
		if ((LogitechController.LB() || (masterShooter.get() * 0.5625) > targetSpeed * 0.9) && !autonomous && !LogitechController.RTrig()) {
			Feeder.start();

		} else {
			Feeder.stop();
			Light.setShoot();
			
		}
	}

	public static void stop() {
		masterShooter.changeControlMode(TalonControlMode.PercentVbus);
		masterShooter.set(0);
		Feeder.lightStop();
	
	}
	
	public static boolean getActive() {
		if ((LogitechController.RB() || getRPM() > 6250)) {
			active = true;
		} else {
			active = false;
		}
		return active;
	}
	
	public static double getRPM() {
		return masterShooter.get() * 0.5626;
	}
	
	public static void setAutonomous(boolean auto) {
		autonomous = auto;
	}
}
