package imu;

import java.util.ArrayList;
import java.util.List;

import maths.EulerRotation;
import maths.EulerRotationMode;
import maths.Quaternion;
import maths.Vec3;

public class IMU {
	
	private Quaternion rot = Quaternion.getForward();
	private Vec3 pos = new Vec3();
	private Vec3 vel = new Vec3();
	private Vec3 gyroOffset = new Vec3();
	private Vec3 accelerometerOffset = new Vec3();
	
	private final Vec3 gyroMultiplier = new Vec3(-1.17, -1.17, -1.17);
	private Vec3 acclererometerMultiplier = new Vec3(1, 1, 1); // used for correcting clock wise to anti clock wise
	private Vec3 lastRawGyro = new Vec3();
	private Vec3 lastFilteredGyro = new Vec3();
	private Vec3 lastRawAccelerometer = new Vec3();
	private Vec3 lastFilteredAccelerometer = new Vec3();
	private Vec3 lastRawMagnetometer = new Vec3();
	private Vec3 lastFilteredMagnetometer = new Vec3();
	private Vec3 magnetometerMultiplier = new Vec3(1,1,1);
	
	private Vec3 prevFilteredAcc = new Vec3(0, 0, 1);
	
	private static final double GRAVITY = 9.807;
	
	private long lastGyroTime = 0;
	private long lastAccelerometerTime = 0;
	
	private List<Vec3> rawGyroVals = new ArrayList<>();
	private List<Vec3> rawAccelerometerVals = new ArrayList<>();
	
	private List<IMUChangeListener> listeners = new ArrayList<>();
	
	public IMU() {
		super();
	}
	
	private void triggerListeners() {
		for (IMUChangeListener listener : listeners) {
			listener.imuChanged();
		}
	}
	
	public void updateGyro(double x, double y, double z) {
		Vec3 unfiltered = new Vec3(x, y, z);
		lastRawGyro = unfiltered;
		rawGyroVals.add(unfiltered);
		
		Vec3 filtered = unfiltered.clone().subtract(gyroOffset);
		filtered.multiply(gyroMultiplier);
		lastFilteredGyro = filtered;
		
		long now = System.nanoTime();
		if(lastGyroTime != 0 ) {
			double elapsedSeconds = (now - lastGyroTime) / 1000000000D;
			EulerRotation localRotation = new EulerRotation(filtered.clone().multiply(elapsedSeconds).toRadians(), EulerRotationMode.ZYX_EULER);
			Quaternion localRotationQuaternion = new Quaternion(localRotation);
			rot.normalise();
			rot.multiply(localRotationQuaternion);
		}
		lastGyroTime = now;
		triggerListeners();
	}
	
	public void updateAccelerometer(double x, double y, double z) {
		/**
		 * Setup
		 */
		long now = System.nanoTime();
		rawAccelerometerVals.add(lastRawAccelerometer);
		lastRawAccelerometer = new Vec3(x, y, z);
		Vec3 filtered = new Vec3(x, y, z).subtract(accelerometerOffset).multiply(acclererometerMultiplier);
		filtered = lowPassFilter(prevFilteredAcc, filtered, 0.5);
		prevFilteredAcc = filtered.clone();
		lastFilteredAccelerometer = filtered;
		
		/**
		 * rotation
		 */
		double roll = Math.atan2(filtered.y, filtered.z);
		double pitch = Math.atan2(-filtered.x, Math.sqrt(filtered.y*filtered.y + filtered.z*filtered.z));
		Quaternion accRot = new Quaternion(new EulerRotation(roll, pitch, -rot.toEulerZYX().z, EulerRotationMode.ZYX_EULER));
		rot.normalise();
		rot.calibrate();
		if(filtered.getLength() < 1.4) {
			rot = Quaternion.lerp(accRot, rot, 0.99);
		}
		
		/**
		 * Velocity
		 */
		Vec3 up = new Vec3(0, 0, 1);
		rot.rotate(up);
		Vec3 right = new Vec3(0, 1, 0);
		rot.rotate(right);
		Vec3 forward = new Vec3(1, 0, 0);
		rot.rotate(forward);
		
		if(lastAccelerometerTime != 0) {
			double seconds = (now - lastAccelerometerTime) / 1000000000D;
			Vec3 acceleration = new Vec3(0, 0, -1);// 1G
			acceleration.add(filtered);
			vel.add(acceleration.multiply(Math.pow(seconds, 2)).multiply(1));// ms^2
			pos.add(vel.clone().multiply(seconds));
		}
		
//		rot = qPitch;
		lastAccelerometerTime = now;
		triggerListeners();
	}
	
	public void updateMagnetometer(double x, double y, double z) {
		Vec3 unfiltered = new Vec3(x, y, z);
		lastRawMagnetometer = unfiltered;
//		Vec3 filtered = unfiltered.clone().multiply(magnetometerMultiplier);
		Vec3 filtered = unfiltered.clone().multiply(new Vec3(1,1,1));
		lastFilteredMagnetometer = filtered;
		
		EulerRotation r = rot.toEulerZYX();
//		
//		double xh = x * Math.cos(r.getPitch()) + y * Math.sin(r.getRoll()) * Math.sin(r.getPitch()) - z * Math.cos(r.getRoll()) * Math.sin(r.getPitch());
//		
//		double yh = y * Math.cos(r.getRoll()) + z * Math.sin(r.getRoll());
//		
//		double yaw = Math.atan2((float)(xh),(float)(yh));
		
//		System.out.println(yaw);
		
		
		double X_h = (double)filtered.x*Math.cos(r.getPitch()) + (double)filtered.y*Math.sin(r.getRoll())*Math.sin(r.getPitch()) + (double)filtered.z*Math.cos(r.getRoll())*Math.sin(r.getPitch());
		double Y_h = (double)filtered.y*Math.cos(r.getRoll()) - (double)filtered.z*Math.sin(r.getRoll());
		double yaw = Math.atan2(Y_h, X_h);
		if(yaw < 0) {	/* Convert yaw in the range (0, 2pi) */
			yaw = 2 * Math.PI + yaw;
		}
//		System.out.println(Math.round(Math.toDegrees(yaw)));
		
//		rot.normalise();
//		rot.calibrate();
		
		Quaternion mq = new Quaternion(new EulerRotation(r.x, -r.y, yaw, EulerRotationMode.ZYX_EULER));
		rot.normalise();
		if(mq.dot(rot) < 0) {
			mq.negate();
			mq.normalise();
		}
		
		rot = Quaternion.lerp(rot, mq, 0);
//		rot.setFromEuler(new EulerRotation(r.x, -r.y, yaw, EulerRotationMode.ZYX_EULER));
		
		triggerListeners();
	}
	
	private static Vec3 lowPassFilter(Vec3 prevFiltered, Vec3 now, double smoothing) {
		return now.clone().multiply(smoothing).add(prevFiltered.clone().multiply(1 - smoothing));
	}
	
	public EulerRotation getRotation() { 
		return rot.toEulerZYX();
	}
	
	public Quaternion getRotationQuaternion() {
		return rot; 
	}
	
	public Vec3 getVelocity() {
		return null;
	}
	
	public void calibrate() {
		System.out.println("Calibrating IMU. Using " + Math.min(100, rawGyroVals.size()) + " Cycles");
		rot = Quaternion.getForward();
		pos = new Vec3();
		vel = new Vec3();
		/**
		 * gyro offset
		 */
		int avgLength = Math.min(100, rawGyroVals.size());
		Vec3 sum = new Vec3();
		for (int i = rawGyroVals.size() - avgLength; i < rawGyroVals.size(); i++) {
			sum.add(rawGyroVals.get(i));
		}
		gyroOffset = sum.divide(avgLength);
		System.out.println("gyro offset is " + gyroOffset);
		
		/**
		 * accelerometer offset
		 */
		avgLength = Math.min(100, rawAccelerometerVals.size());
		sum = new Vec3();
		for (int i = rawAccelerometerVals.size() - avgLength; i < rawAccelerometerVals.size(); i++) {
			sum.add(rawAccelerometerVals.get(i));
		}
		accelerometerOffset = sum.divide(avgLength).subtract(new Vec3(0,0,1));
		System.out.println("accelerometer offset is " + accelerometerOffset);
	}
	
	public boolean addIMUChangeListener(IMUChangeListener listener) {
		return listeners.add(listener);
	}
	public boolean removeIMUChangeListener(IMUChangeListener listener) {
		return listeners.remove(listener);
	}

	public Vec3 getLastRawGyro() {
		return lastRawGyro;
	}

	public void setLastRawGyro(Vec3 lastRawGyro) {
		this.lastRawGyro = lastRawGyro;
	}

	public Vec3 getLastFilteredGyro() {
		return lastFilteredGyro;
	}

	public Vec3 getLastRawAccelerometer() {
		return lastRawAccelerometer;
	}

	public Vec3 getLastRawMagnetometer() {
		return lastRawMagnetometer;
	}
	
	public Vec3 getLastFilteredMagnetometer() {
		return lastFilteredMagnetometer;
	}
	
	public long getLastGyroTime() {
		return lastGyroTime;
	}

	public Vec3 getLastFilteredAccelerometer() {
		return lastFilteredAccelerometer;
	}

	public Vec3 getPos() {
		return pos;
	}

	public void setLastGyroTime(long lastGyroTime) {
		this.lastGyroTime = lastGyroTime;
	}
}