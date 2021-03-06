package Master;

/**
 * For representing and operating on 3D vectors
 * 
 * @author Nick
 *
 */
public class Vector {
	private double x;
	private double y;
	private double z;
	
	/**
	 * Constructor
	 * 
	 * @param x X component of new vector.
	 * @param y Y component of new vector.
	 * @param z Z component of new vector.
	 */
	public Vector(double x, double y, double z){
		this.x = x;
		this.y = y;
		this.z = z;
	}
	
	/**
	 * @return The x component of the vector.
	 */
	public double getX(){
		return x;
	}
	
	/**
	 * @return The y component of the vector.
	 */
	public double getY(){
		return y;
	}
	
	/**
	 * @return The z component of the vector.
	 */
	public double getZ(){
		return z;
	}
	
	/**
	 * Set the vector's x component.
	 */
	public void setX(double x){
		this.x = x;
	}
	
	/**
	 * Set the vector's y component.
	 */
	public void setY(double y){
		this.y = y;
	}
	
	/**
	 * Set the vector's z component.
	 */
	public void setZ(double z){
		this.z = z;
	}
	
	/**
	 * Get the angle of the vector, when it is projected onto the xy-plane.
	 * 
	 * @return The vector's angle on the xy-plane.
	 */
	public double xyAngleDeg(){
		return Math.atan2(y, x) * 180.0 / Math.PI;
	}
	
	/**
	 * Return the vector rotated on the xy-plane counterclockwise by the given
	 * amount.
	 * 
	 * @param angle Angle to rotate counter-clockwise.
	 * 
	 * @return The rotated vector.
	 */
	public static Vector xyRotation(Vector v, double angle){
		// Math formula found on the web.
		double c = Math.cos(angle);
		double s = Math.sin(angle);
		
		double newX = v.x * c - v.y * s;
		double newY = v.x * s + v.y * c;
		
		return new Vector(newX, newY, v.z);
	}
	
	/**
	 * @return The length of the vector.
	 */
	public double length(){
		return Math.sqrt(x*x + y*y + z*z);
	}
	
	/**
	 * Set the vector to new values.
	 * 
	 * @param x New x coordinate.
	 * @param y New y coordinate.
	 * @param z New z coordinate.
	 */
	public void set(double x, double y, double z){
		this.x = x;
		this.y = y;
		this.z = z;
	}
	
	/**
	 * Divides all components by a number.
	 * 
	 * @param num The divisor.
	 */
	public void divide(double num){
		x /= num;
		y /= num;
		z /= num;
	}
	
	/**
	 * Add two vectors.
	 * 
	 * @param a The first vector.
	 * @param b The second vector.
	 * 
	 * @return The sum of the two vectors.
	 */
	public static Vector add(Vector a, Vector b){
		double x = a.x + b.x;
		double y = a.y + b.y;
		double z = a.z + b.z;
		
		return new Vector(x, y, z);
	}
	
	/**
	 * Substract two vectors.
	 * 
	 * @param a The first vector.
	 * @param b The second vector.
	 * @return The difference (a-b).
	 */
	public static Vector substract(Vector a, Vector b){
		double x = a.x - b.x;
		double y = a.y - b.y;
		double z = a.z - b.z;
		
		return new Vector(x, y, z);
	}
	
	/**
	 * The dot product of two vectors.
	 * 
	 * @param a The first vector.
	 * @param b The second vector.
	 * 
	 * @return The dot product.
	 */
	public static double dot(Vector a, Vector b){
		return a.x * b.x +
			   a.y * b.y +
			   a.z * b.z;
	}
	
	/**
	 * The average of an array of vectors.
	 * 
	 * @param group Group of vectors.
	 * 
	 * @return The vector whose components are the average of all the vectors.
	 */
	public static Vector average(Vector group[]) {
		Vector ave = new Vector(0.0, 0.0, 0.0);
		double length = group.length;
		
		for (int i = 0; i < length; i++) {
			ave = Vector.add(ave, group[i]);
		}
		ave.divide(length);
		
		return ave;
	}
	
	/**
	 * The cosine of the angle between two vectors.
	 * 
	 * @param a The first Vector.
	 * @param b The second Vector.
	 * 
	 * @return The cosine of the angle between a and b.
	 */
	public static double cosTheta(Vector a, Vector b){
		double cosTheta = Vector.dot(a,  b);
		cosTheta =  cosTheta / (a.length() * b.length());
		
		return cosTheta;
	}

	/**
	 * Returns the the angle between two vectors.
	 * 
	 * @param a The first Vector.
	 * @param b The second Vector.
	 * 
	 * @return The angle between a and b.
	 */
	public static double theta(Vector a, Vector b){
		double cosTheta = cosTheta(a, b);
		
		return Math.acos(cosTheta);
	}
}
