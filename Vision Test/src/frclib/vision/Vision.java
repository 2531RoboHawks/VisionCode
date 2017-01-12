package frclib.vision;

import java.util.ArrayList;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;

public class Vision {

	private int min1 = 0, min2 = 0, min3 = 0;
	private int max1 = 0, max2 = 0, max3 = 0;

	private UsbCamera cam;
	private CvSink in1;

	public Vision() {
		cam = CameraServer.getInstance().startAutomaticCapture();
		cam.setResolution(640, 480);
		in1 = CameraServer.getInstance().getVideo("cam1");
		out = CameraServer.getInstance().putVideo("vision", 640, 480);
	}

	public void start() {
	}

	public void updateFeed() {
		Mat m = new Mat();
		in.grabFrame(m);
		out.putFrame(m);
	}

	public void setColor(int min1, int max1, int min2, int max2, int min3, int max3) {
		this.min1 = min1;
		this.max1 = max1;
		this.min2 = min2;
		this.max2 = max2;
		this.min3 = min3;
		this.max3 = max3;
	}

	public ArrayList<Rect> HLSgetBlobs(Mat src) {
		Mat mat = src.clone();
		ArrayList<Rect> blobs = new ArrayList<Rect>();
		ArrayList<MatOfPoint> c = new ArrayList<MatOfPoint>();
		Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2HLS);
		Core.inRange(mat, new Scalar(min1, min2, min3), new Scalar(max1, max2, max3), mat);
		Imgproc.findContours(mat, c, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
		for (int i = 0; i < c.size(); i++) {
			MatOfPoint mop = c.get(i);
			if (mop != null) {
				blobs.add(Imgproc.boundingRect(mop));
			}
		}
		return blobs;
	}

	public ArrayList<Rect> HSVgetBlobs(Mat src) {
		Mat mat = src.clone();
		ArrayList<Rect> blobs = new ArrayList<Rect>();
		ArrayList<MatOfPoint> c = new ArrayList<MatOfPoint>();
		Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2HSV);
		Core.inRange(mat, new Scalar(min1, min2, min3), new Scalar(max1, max2, max3), mat);
		Imgproc.findContours(mat, c, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
		for (int i = 0; i < c.size(); i++) {
			MatOfPoint mop = c.get(i);
			if (mop != null) {
				blobs.add(Imgproc.boundingRect(mop));
			}
		}
		return blobs;
	}

	public ArrayList<Rect> RGBgetBlobs(Mat src) {
		Mat mat = src.clone();
		ArrayList<Rect> blobs = new ArrayList<Rect>();
		ArrayList<MatOfPoint> c = new ArrayList<MatOfPoint>();
		Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2RGB);
		Core.inRange(mat, new Scalar(min1, min2, min3), new Scalar(max1, max2, max3), mat);
		Imgproc.findContours(mat, c, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
		for (int i = 0; i < c.size(); i++) {
			MatOfPoint mop = c.get(i);
			if (mop != null) {
				blobs.add(Imgproc.boundingRect(mop));
			}
		}
		return blobs;
	}

	public ArrayList<Rect> HLSgetBlobs() {
		Mat mat = getImage();
		ArrayList<Rect> blobs = new ArrayList<Rect>();
		ArrayList<MatOfPoint> c = new ArrayList<MatOfPoint>();
		Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2HLS);
		Core.inRange(mat, new Scalar(min1, min2, min3), new Scalar(max1, max2, max3), mat);
		Imgproc.findContours(mat, c, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
		for (int i = 0; i < c.size(); i++) {
			MatOfPoint mop = c.get(i);
			if (mop != null) {
				blobs.add(Imgproc.boundingRect(mop));
			}
		}
		return blobs;
	}

	public ArrayList<Rect> HSVgetBlobs() {
		Mat mat = getImage();
		ArrayList<Rect> blobs = new ArrayList<Rect>();
		ArrayList<MatOfPoint> c = new ArrayList<MatOfPoint>();
		Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2HSV);
		Core.inRange(mat, new Scalar(min1, min2, min3), new Scalar(max1, max2, max3), mat);
		Imgproc.findContours(mat, c, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
		for (int i = 0; i < c.size(); i++) {
			MatOfPoint mop = c.get(i);
			if (mop != null) {
				blobs.add(Imgproc.boundingRect(mop));
			}
		}
		return blobs;
	}

	public ArrayList<Rect> RGBgetBlobs() {
		Mat mat = getImage();
		ArrayList<Rect> blobs = new ArrayList<Rect>();
		ArrayList<MatOfPoint> c = new ArrayList<MatOfPoint>();
		Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2RGB);
		Core.inRange(mat, new Scalar(min1, min2, min3), new Scalar(max1, max2, max3), mat);
		Imgproc.findContours(mat, c, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
		for (int i = 0; i < c.size(); i++) {
			MatOfPoint mop = c.get(i);
			if (mop != null) {
				blobs.add(Imgproc.boundingRect(mop));
			}
		}
		return blobs;
	}

	public static double getDistance(Rect rect, double fov, int objectwidth, int imagewidth) {
		if (rect != null) {
			double d = objectwidth * imagewidth / (2 * rect.width * Math.tan(fov));
			return d;
		}
		return 0;
	}

	private CvSink in;

	public Mat getImage() {
		CameraServer.getInstance().addCamera(cam);
		Mat mat = new Mat();
		// in.grabFrame(mat);
		CameraServer.getInstance().getVideo().grabFrame(mat);
		CameraServer.getInstance().removeCamera(cam.getName());
		return mat;
	}

	public Mat getImage(String name) {
		Mat mat = new Mat();
		//in.grabFrame(mat);
		CameraServer.getInstance().getVideo(name).grabFrame(mat);
		// CameraServer.getInstance().removeCamera(cam.getName());
		return mat;
	}

	private CvSource out;

	public void putImage(String name, Mat mat) {
		CameraServer.getInstance().putVideo(name, 640, 480).putFrame(mat);
		// out.putFrame(mat);
	}

	public Mat showBlobs(Mat src, ArrayList<Rect> blobs, Scalar color) {
		Mat mat = src.clone();
		for (int i = 0; i < blobs.size(); i++) {
			Rect r = blobs.get(i);
			if (r != null) {
				Imgproc.rectangle(mat, r.tl(), r.br(), color, 2);
			}
		}
		return mat;
	}
}
