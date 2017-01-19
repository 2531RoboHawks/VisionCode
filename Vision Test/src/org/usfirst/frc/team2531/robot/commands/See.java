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

	private PID t = new PID(0.001, 0, 0, 320);
	private PID m = new PID(0.005, 0, 0, 240);
	private double pow = 0;
	private double fwd = 0;
	private double lastpos = 0;

	public See() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.drive);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		System.out.println("-> See");
		t.setOutputLimits(-0.5, 0.5);
		m.setOutputLimits(-0.3, 0.3);
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
		if (!l.isEmpty()) {
			x /= l.size();
			lastpos = x;
			mat = Robot.cam0.showBlobs(mat, l, new Scalar(0, 255, 0));
			Imgproc.line(mat, new Point(x, 0), new Point(x, 480), new Scalar(0, 255, 0), 5);
			pow = t.compute(x);
			fwd = m.compute(y);
			Robot.drive.axisdrive(0, fwd, -pow);
			Robot.cam0.putImage(mat);
		} else {
			Robot.cam0.showLive();
			pow = t.compute(lastpos);
			Robot.drive.axisdrive(0, 0, -pow);
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
