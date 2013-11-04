package Slave;

//for creating and operating vectors
public class Vector {
	// Variables
	private double x;
	private double y;
	private double z;
	
	public Vector(double x, double y, double z){
		this.x = x;
		this.y = y;
		this.z = z;
	}
	
	public double getX(){
		return x;
	}
	
	public double getY(){
		return y;
	}
	
	public double getZ(){
		return z;
	}
	
	/* Length of vector
	 */
	public double length(){
		return Math.sqrt(x*x + y*y + z*z);
	}
	
	/* Divide all components by a number
	 */
	public void divide(double num){
		x /= num;
		y /= num;
		z /= num;
	}
	
	/* Sum of two vectors
	 */
	public static Vector add(Vector a, Vector b){
		double x = a.x + b.x;
		double y = a.y + b.y;
		double z = a.z + b.z;
		
		return new Vector(x, y, z);
	}
	
	/* Difference of two vectors
	 */
	public static Vector substract(Vector a, Vector b){
		double x = a.x - b.x;
		double y = a.y - b.y;
		double z = a.z - b.z;
		
		return new Vector(x, y, z);
	}
	
	/* Dot product of two vectors
	 */
	public static double dot(Vector a, Vector b){
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}
	
	/* Returns Vec3 whose components are the average of all the Vec3's
	 * components.
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
