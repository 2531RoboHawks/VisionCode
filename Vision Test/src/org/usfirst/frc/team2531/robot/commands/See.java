package org.usfirst.frc.team2531.robot.commands;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team2531.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import frclib.pid.PID;

/**
 *
 */
public class See extends Command {

	private PID m = new PID(0.0015, 0, 0, 320);
	private double last = 0;

	public See() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.drive);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		System.out.println("-> See");
		m.setOutputLimits(-0.5, 0.5);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Mat mat = Robot.cam0.getImage();
		Robot.cam0.setColor(100, 255, 0, 50, 0, 50);
		ArrayList<Rect> l = Robot.cam0.filterArea(Robot.cam0.RGBgetBlobs(mat), 200);
		int t = 0;
		for (int i = 0; i < l.size(); i++) {
			Rect r = l.get(i);
			if (r != null) {
				t = r.x + (r.width / 2);
			}
		}
		if (!l.isEmpty()) {
			t /= l.size();
			mat = Robot.cam0.showBlobs(mat, l, new Scalar(0, 255, 0));
			Imgproc.line(mat, new Point(t, 0), new Point(t, 480), new Scalar(0, 255, 0), 5);
			double pow = m.compute(t);
			Robot.drive.axisdrive(0, 0, -pow);
			if (-pow > 0) {
				last = 0.4;
			} else {
				last = -0.4;
			}
			Robot.cam0.putImage(mat);
		} else {
			Robot.cam0.showLive();
			Robot.drive.axisdrive(0, 0, -last);
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.drive.axisdrive(0, 0, 0);
		System.out.println("-! See");
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
