package Master;

import java.lang.Math;


public class MyMath {
	/**
	 * The smallest angle between two given angles. 
	 * 
	 * @param angleA
	 * @param angleB
	 * @return The angle between angleA and angleB.
	 */
	public static double angleBetweenDeg(double angleA, double angleB){
		double between = angleA - angleB;
		
		if(between > 180.0){
			between = 360.0 - between;
		}
		else if(between < -180.0){
			between = 360.0 + between;
		}
		else if(between < 0.0){
			between = -between;
		}
		
		return between;
	}
	
	/**
	 * Find the angle needed to get to the destination angle.
	 * <p>
	 * This angle is in the range [-180.0, 180.0]. You add it to currentAngle
	 * in order to obtain destinationAngle.
	 * 
	 * @param currentAngle
	 * @param destinationAngle
	 * @return The angle needed to get from the current angle to the destination, in the range [-180.0, 180.0].
	 */
	public static double correctionDeg(double currentAngle, double destinationAngle){
		double correctionAngle = destinationAngle - currentAngle;
		
		if(correctionAngle > 180.0){
			correctionAngle = correctionAngle - 360.0;
		}
		else if(correctionAngle < -180.0){
			correctionAngle = correctionAngle + 360.0; 
		}
		
		return correctionAngle;
	}
	
	/**
	 * Find the equivalent angle in the range [0.0, 360.0].
	 * 
	 * @param angle Angle in degrees.
	 * 
	 * @return The angle in the range [0.0, 360.0].
	 */
	public static double fixAngleDeg(double angle){
		while(angle < 0.0){
			angle += 360.0;
		}
		
		angle %= 360.0;
		
		return angle;
	}
	
	/**
	 * The distance between the points (x1, y1) and (x2, y2).
	 * 
	 * @return The distance.
	 */
	public static double distance(double x1, double y1, double x2, double y2){
		double dx = x2 - x1;
		double dy = y2 - y1;
		
		return Math.sqrt(dx * dx + dy * dy);
	}
}
