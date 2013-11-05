package Slave;

//for creating and operating vectors
public class Vector {
	// Variables
	private double x;
	private double y;
	private double z;
	
	/**
	 * Constructor
	 * @param x X component of new vector
	 * @param y Y component of new vector
	 * @param z Z component of new vector
	 */
	public Vector(double x, double y, double z){
		this.x = x;
		this.y = y;
		this.z = z;
	}
	
	/**
	 * @return double - The x component of the vector
	 */
	public double getX(){
		return x;
	}
	
	/**
	 * @return double - The y component of the vector
	 */
	public double getY(){
		return y;
	}
	
	/**
	 * @return double - The z component of the vector
	 */
	public double getZ(){
		return z;
	}
	
	/**
	 * @return double - The length of the vector
	 */
	public double length(){
		return Math.sqrt(x*x + y*y + z*z);
	}
	
	/**
	 * Divides all components by a number
	 * @param num The number to divide all components by
	 * @return void
	 */
	public void divide(double num){
		x /= num;
		y /= num;
		z /= num;
	}
	
	/**
	 * @param a The first vector
	 * @param b The second vector
	 * @return Vector - The sum of the two vectors
	 */
	public static Vector add(Vector a, Vector b){
		double x = a.x + b.x;
		double y = a.y + b.y;
		double z = a.z + b.z;
		
		return new Vector(x, y, z);
	}
	
	/**
	 * @param a The first vector
	 * @param b The second vector
	 * @return Vector - The difference of a and b (a-b)
	 */
	public static Vector substract(Vector a, Vector b){
		double x = a.x - b.x;
		double y = a.y - b.y;
		double z = a.z - b.z;
		
		return new Vector(x, y, z);
	}
	
	/**
	 * @param a The first vector
	 * @param b The second vector
	 * @return Vector - The dot product of the two vectors
	 */
	public static double dot(Vector a, Vector b){
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}
	
	/**
	 * @param group List of vectors 
	 * @return Vector - A vector whose components are the average of all the components in the list
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
}
