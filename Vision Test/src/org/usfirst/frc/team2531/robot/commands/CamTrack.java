package org.usfirst.frc.team2531.robot.commands;

import java.util.ArrayList;

import org.usfirst.frc.team2531.robot.Robot;

import com.ni.vision.NIVision.Rect;

import edu.wpi.first.wpilibj.command.Command;
import frclib.pid.PID;
import frclib.vision.Camera;

/**
 *
 */
public class CamTrack extends Command {
	PID pid = new PID(0.1, 0, 0, 0);
	double out = 0;
	double minarea = 20;

	public CamTrack(Camera c) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.drive);
		pid.setOutputLimits(-0.8, 0.8);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		ArrayList<Rect> blobs = Robot.vision.getBlobs();
		if (blobs != null) {
			for (int i = 0; i < blobs.size(); i++) {
				Rect r = blobs.get(i);
				if (r.width * r.height < minarea) {
					blobs.remove(i);
				}
			}
			int pos = 0;
			for (int i = 0; i < blobs.size(); i++) {
				Rect r = blobs.get(i);
				pos += (r.left - r.width / 2);
			}
			if (blobs.size() > 0) {
				pos /= blobs.size();
				pos-=640;
				out = pos/640;
			}else{
				out = 0;
			}
		} else {
			out = 0;
		}
		Robot.drive.TankDrive(out, -out);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
