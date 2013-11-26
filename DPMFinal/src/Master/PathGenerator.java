package Master;

public class PathGenerator {

	private Odometer odometer;
	
	private double BORDER_DIST = 10.0;		//border distance for checking if a point is in an area
	private double GREEN_BORDER_DIST = 15.0;	//must be larger than BORDER_DIST, for travelling to border point
	
	private double gx0=60;			//green zone left x component
	private double gx1=120;			//green zone right x component
	private double gy0=60;			//green zone lower y component
	private double gy1=180;			//green zone upper y component
	
	private double rx0=600;			//red zone left x component
	private double rx1=900;			//red zone right x component
	private double ry0=600;			//red zone lower y component
	private double ry1=900;			//red zone upper y component
	
	private double wx0 = -30.0;			//left wall
	private double wx1 = 330.0;			//right wall
	private double wy0 = -30.0;			//lower wall
	private double wy1 = 330.0;			//upper wall
	

	
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
	
	public double[] closestRedCorner(double x0, double y0){
		double[] point  = new double[2];
		
		double d0 = Math.sqrt( (Math.pow( Math.abs(rx0 - BORDER_DIST - x0),2 )) + (Math.pow(Math.abs(ry0 - BORDER_DIST - y0),2))); 
		double d1 = Math.sqrt( (Math.pow( Math.abs(rx1 + BORDER_DIST - x0),2 )) + (Math.pow(Math.abs(ry0 - BORDER_DIST - y0),2))); 
		double d2 = Math.sqrt( (Math.pow( Math.abs(rx0 - BORDER_DIST - x0),2 )) + (Math.pow(Math.abs(ry1 + BORDER_DIST - y0),2))); 
		double d3 = Math.sqrt( (Math.pow( Math.abs(rx1 + BORDER_DIST - x0),2 )) + (Math.pow(Math.abs(ry1 + BORDER_DIST - y0),2))); 

		if(d0 < 10){
			//too close to this point, assumed already here
			if (d1 <= d2 && d1 <= d3){
				point[0] = rx1 + BORDER_DIST;
				point[1] = ry0 - BORDER_DIST;
			}else if (d2 <= d1 && d2 <= d3){
				point[0] = rx0 - BORDER_DIST;
				point[1] = ry1 + BORDER_DIST;
			}else{
				point[0] = rx1 + BORDER_DIST;
				point[1] = ry1 + BORDER_DIST;
			}
		}else if (d1 < 10){
			if (d0 <= d2 && d0 <= d3){
				point[0] = rx0 - BORDER_DIST;
				point[1] = ry0 - BORDER_DIST;
			}else if (d2 <= d0 && d2 <= d3){
				point[0] = rx0 - BORDER_DIST;
				point[1] = ry1 + BORDER_DIST;
			}else{
				point[0] = rx1 + BORDER_DIST;
				point[1] = ry1 + BORDER_DIST;
			}
		}else if (d2 < 10){
			if (d0 <= d1 && d0 <= d3){
				point[0] = rx0 - BORDER_DIST;
				point[1] = ry0 - BORDER_DIST;
			}else if (d1 <= d0 && d1 <= d3){
				point[0] = rx1 + BORDER_DIST;
				point[1] = ry0 - BORDER_DIST;
			}else{
				point[0] = rx1 + BORDER_DIST;
				point[1] = ry1 + BORDER_DIST;
			}
		}else if (d3 < 10){
			if (d0 <= d1 && d0 <= d2){
				point[0] = rx0 - BORDER_DIST;
				point[1] = ry0 - BORDER_DIST;
			}else if (d1 <= d0 && d1 <= d2){
				point[0] = rx1 + BORDER_DIST;
				point[1] = ry0 - BORDER_DIST;
			}else{
				point[0] = rx0 - BORDER_DIST;
				point[1] = ry1 + BORDER_DIST;
			}
		}else{
			if (d0 <= d1 && d0 <= d2 && d0 <= d3){
				point[0] = rx0 - BORDER_DIST;
				point[1] = ry0 - BORDER_DIST;
			}else if (d1 <= d0 && d1 <= d2 && d1 <= d3){
				point[0] = rx1 + BORDER_DIST;
				point[1] = ry0 - BORDER_DIST;
			}else if (d2 <= d0 && d2 <= d1 && d2 <= d3){
				point[0] = rx0 - BORDER_DIST;
				point[1] = ry1 + BORDER_DIST;
			}else{
				point[0] = rx1 + BORDER_DIST;
				point[1] = ry1 + BORDER_DIST;
			}
		}
		return point;

	}
	
	public double[] generateSquarePath(double x, double y, double depositAngle){
		double[] path = new double[6];	//[x1, y1, x2, y2]
		double[] tempPoint = new double[2];
		double borderX;
		double borderY;
		
		//initialized to invalid point
		for(int i=0; i < 6; i++){
			path[i] = -100;
		}
		
		if(checkPointsInPath(odometer.getX(), odometer.getY(), x, y)){
			path[0] = x;
			path[1] = y;
		}else{
			tempPoint = closestRedCorner(odometer.getX(), odometer.getY());
			path[0] = tempPoint[0];
			path[1] = tempPoint[1];
			
			if(checkPointsInPath(path[0], path[1], x, y)){
				path[2] = x;
				path[3] = y;
			}else{
				tempPoint = closestRedCorner(path[0], path[1]);
				path[2] = tempPoint[0];
				path[3] = tempPoint[1];
				
				if(checkPointsInPath(path[2], path[3], x, y)){
					path[4] = x;
					path[5] = y;
				}else{
					tempPoint = closestRedCorner(path[2], path[3]);
					path[4] = tempPoint[0];
					path[5] = tempPoint[1];
				}
				
			}
		}
		
		return path;

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
	    if((x <= wx0 + border) || (x >= wx1 - border) || (y <= wy0 + border) || (y >= wy1 - border)){
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
	
	

	/*public double minimumOfFour(double a, double b, double c, double d){
		if(a < b && a < c && a < d){
			return a;
		}else if (b < a && b < c && b < d){
			return b;
		}else if (c < a && c < b && c < d){
			return c;
		}else{
			return d;
		}
	}
	
	public double minimumOfThree(double a, double b, double c){
		if(a <= b && a <= c){
			return a;
		}else if (b <= a && b <= c ){
			return b;
		}else{
			return c;
		}
	}*/

	
}
