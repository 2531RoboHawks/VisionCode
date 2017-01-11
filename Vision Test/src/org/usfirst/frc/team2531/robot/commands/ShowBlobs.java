package org.usfirst.frc.team2531.robot.commands;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.usfirst.frc.team2531.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ShowBlobs extends Command {

	public ShowBlobs() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		System.out.println("-> ShowBlobs");
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Mat m = RobotMap.vision.getImage();
		RobotMap.vision.setColor(0, 255, 0, 20, 0, 255);
		RobotMap.vision.putImage("camera",
				RobotMap.vision.showBlobs(m, RobotMap.vision.HLSgetBlobs(m), new Scalar(0, 255, 0)));
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		System.out.println("-! ShowBlobs");
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
