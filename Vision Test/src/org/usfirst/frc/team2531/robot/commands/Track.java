package org.usfirst.frc.team2531.robot.commands;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.usfirst.frc.team2531.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import frclib.pid.PID;

/**
 *
 */
public class Track extends Command {

	private PID rot = new PID(0.001, 0, 0, 320);

	private double rot_output = 0;
	private int last_x = 0;
	private int last_y = 0;

	public Track() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.drive);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		System.out.println("-> Track");
		rot.setOutputLimits(-0.3, 0.3);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Mat mat = Robot.cam0.getImage();
		Robot.cam0.setColor(100, 255, 0, 50, 0, 50);
		ArrayList<Rect> l = Robot.cam0.filterArea(Robot.cam0.RGBgetBlobs(mat), 200);
		int x = 0;
		int y = 0;
		for (int i = 0; i < l.size(); i++) {
			Rect r = l.get(i);
			if (r != null) {
				x = r.x + (r.width / 2);
				y = r.y + (r.height / 2);
			}
		}
		Robot.cam0.putImage(Robot.cam0.showBlobs(mat, l, new Scalar(0, 255, 0)));
		if (!l.isEmpty()) {
			x /= l.size();
			y /= l.size();
			last_x = x;
			last_y = y;
			rot_output = -rot.compute(x);
			if (!rot.onTarget()) {
				if (rot_output > 0) {
					if (rot_output < 0.1) {
						rot_output = 0.1;
					}
				} else {
					if (rot_output > -0.1) {
						rot_output = -0.1;
					}
				}
			}
			Robot.drive.axisdrive(0, 0, rot_output);
		} else {
			rot_output = -rot.compute(last_x);
			if (!rot.onTarget()) {
				if (rot_output > 0) {
					if (rot_output < 0.1) {
						rot_output = 0.1;
					}
				} else {
					if (rot_output > -0.1) {
						rot_output = -0.1;
					}
				}
			}
			Robot.drive.axisdrive(0, 0, rot_output);
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return rot.onTarget();
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.drive.axisdrive(0, 0, 0);
		System.out.println("-! Track");
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
