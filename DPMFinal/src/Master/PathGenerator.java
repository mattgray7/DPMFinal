package Master;

public class PathGenerator {

	private Odometer odometer;

	private double BORDER_DIST = 15.0;                //border distance for checking if a point is in an area
	private double GREEN_BORDER_DIST = 18.0;        //must be larger than BORDER_DIST, for traveling to border point

	private double gx0=150;                        //green zone left x component
	private double gx1=180;                        //green zone right x component
	private double gy0=150;                        //green zone lower y component
	private double gy1=180;                        //green zone upper y component

	private double rx0=60;                        //red zone left x component
	private double rx1=90;                        //red zone right x component
	private double ry0=60;                        //red zone lower y component
	private double ry1=90;                        //red zone upper y component

	private double wx0 = -30.0;                        //left wall
	private double wx1 = 210.0;                        //right wall
	private double wy0 = -30.0;                        //lower wall
	private double wy1 = 210.0;                        //upper wall



	public PathGenerator(Odometer odo) {
		this.odometer = odo;
	}

	public double[] calculateBorderPoint(){
		double x = odometer.getX();
		double y = odometer.getY();
		double[] point = new double[2];
		if(x <= gx0 && y <= gy0){
			point[0] = gx0 - GREEN_BORDER_DIST;
			point[1] = gy0 - GREEN_BORDER_DIST;
		}else if (x <= gx0 && y >= gy0){
			point[0] = gx0 - GREEN_BORDER_DIST;
			point[1] = gy1 + GREEN_BORDER_DIST;
		}else if (x >= gx0 && y <= gy0){
			point[0] = gx1 + GREEN_BORDER_DIST;
			point[1] = gy0 - GREEN_BORDER_DIST;
		}else{
			point[0] = gx1 + GREEN_BORDER_DIST;
			point[1] = gy1 + GREEN_BORDER_DIST;
		}
		return point;
	}


	public double[][] findClosestCorner() {
		double[] corner = new double [2];
		double [][] corners = new double [4][2];
		BORDER_DIST += 3;
		if(Math.abs(rx1 + BORDER_DIST - odometer.getX()) < Math.abs(rx0 - BORDER_DIST - odometer.getX())) {
			corner[0] = rx1 + BORDER_DIST;
		}
		else {
			corner[0] = rx0 - BORDER_DIST;
		}

		if(Math.abs(ry1 + BORDER_DIST- odometer.getY()) < Math.abs(ry0 - BORDER_DIST - odometer.getY())) {
			corner[1] = ry1 + BORDER_DIST;
		}
		else {
			corner[1] = ry0 - BORDER_DIST;
		}
		//Check which corner is closest and then assign route for visiting each corner
		if (corner[0] == rx0 - BORDER_DIST && corner[1] == ry0 - BORDER_DIST) {
			corners[0][0] = rx0 - BORDER_DIST;
			corners[0][1] = ry0 - BORDER_DIST;
			corners[1][0] = rx1 + BORDER_DIST;
			corners[1][1] = ry0 - BORDER_DIST;
			corners[2][0] = rx1 + BORDER_DIST; 
			corners[2][1] = ry1 + BORDER_DIST;
			corners[3][0] = rx0 - BORDER_DIST;
			corners[3][1] = ry1 + BORDER_DIST;
		}
		else if (corner[0] == rx1 + BORDER_DIST&& corner[1] == ry0 - BORDER_DIST) {
			corners[0][0] = rx1 + BORDER_DIST;
			corners[0][1] = ry0 - BORDER_DIST;
			corners[1][0] = rx1 + BORDER_DIST;
			corners[1][1] = ry1 + BORDER_DIST;
			corners[2][0] = rx0 - BORDER_DIST; 
			corners[2][1] = ry1 + BORDER_DIST;
			corners[3][0] = rx0 - BORDER_DIST;
			corners[3][1] = ry0 - BORDER_DIST;
		}
		else if (corner[0] == rx1 + BORDER_DIST&& corner[1] == ry1 + BORDER_DIST) {
			corners[0][0] = rx1 + BORDER_DIST;
			corners[0][1] = ry1 + BORDER_DIST;
			corners[1][0] = rx0 - BORDER_DIST;
			corners[1][1] = ry1 + BORDER_DIST;
			corners[2][0] = rx0 - BORDER_DIST; 
			corners[2][1] = ry0 - BORDER_DIST;
			corners[3][0] = rx1 + BORDER_DIST;
			corners[3][1] = ry0 - BORDER_DIST;
		} 
		else {
			corners[0][0] = rx0 - BORDER_DIST;
			corners[0][1] = ry1 + BORDER_DIST;
			corners[1][0] = rx0 - BORDER_DIST;
			corners[1][1] = ry0 - BORDER_DIST;
			corners[2][0] = rx1 + BORDER_DIST; 
			corners[2][1] = ry0 - BORDER_DIST;
			corners[3][0] = rx1 + BORDER_DIST;
			corners[3][1] = ry1 + BORDER_DIST;
		}
		BORDER_DIST -= 3;
		return corners;
	} 


	public Boolean checkPointAhead(double angle, int distance){
		double nx = 0;
		double ny = 0;
		nx = distance*Math.cos(angle*Math.PI/180) + odometer.getX();
		ny = distance*Math.sin(angle*Math.PI/180) + odometer.getY();
		return checkPoint(nx, ny, BORDER_DIST);
	}



	public boolean checkPointsInPath(double x0, double y0, double x1, double y1) {

		double currentX = x0;
		double startX = currentX;
		double currentY = y0;
		double startY = currentY;
		double heading = Math.atan2(y1 - y0, x1 - x0) * (180.0/ Math.PI);

		//Check if any point along path intersects the red zone

		if((x1 - currentX) > 0){
			while(currentX <= x1){
				currentY = (currentX - startX) * Math.tan(heading * Math.PI/180) + startY;
				if(!checkPoint(currentX, currentY, BORDER_DIST)){
					return false;
				}else{
					currentX++;
				}
			}
		}else if ((x1 - currentX) < 0){
			while(currentX >= x1){
				currentY = (startX - currentX) * Math.tan((180.0 - heading) * Math.PI/180) + startY;
				if (!checkPoint(currentX, currentY, BORDER_DIST)){
					return false;
				}else{
					currentX--;
				}
			}

		}else if (Math.abs(x1 - currentX) < 1){
			if(y1 > odometer.getY()){
				while(currentY <= y1){
					if (!checkPoint(currentX, currentY, BORDER_DIST)){
						return false;
					}else{
						currentY++;
					}
				}
			}else{
				while(currentY >= y1){
					if (!checkPoint(currentX, currentY, BORDER_DIST)){
						return false;
					}else{
						currentY--;
					}
				}
			}
		}
		return true;
	}




	public boolean checkPoint(double x, double y, double border) {
		//check if next point is within a wall
		if((x <= wx0 + 28) || (x >= wx1 - 28) || (y <= wy0 + 28) || (y >= wy1 - 28)){
			return false;
		}

		//check if next point is within red zone threshold
		if ((x >= rx0 - border) && (x <= rx1 + border) && (y >= ry0 - border) && (y <= ry1 + border)){
			return false;
		}


		//check if next point is within green zone threshold
		if ((x >= gx0 - border) && (x <= gx1 + border) && (y >= gy0 - border) && ( y <= gy1 + border)){
			return false;
		}


		return true;
	}

	public void setZones(int[] green, int[] red){
		this.gx0 = (double)green[0] * 30.0;
		this.gy0 = (double)green[1] * 30.0;
		this.gx1 = (double)green[2] * 30.0;
		this.gy1 = (double)green[3] * 30.0;

		this.rx0 = (double)red[0] * 30.0;
		this.ry0 = (double)red[1] * 30.0;
		this.rx1 = (double)red[2] * 30.0;
		this.ry1 = (double)red[3] * 30.0;
	}






}