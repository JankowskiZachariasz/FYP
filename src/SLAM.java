import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

public class SLAM {
	
	ArrayList<double[]> featureMatches=null, fetureCopied=null;//prevIndex,prevX,prevZ,,currIndex,currX,currZ,distance
	static double PROXIMITY_THRESHOLD = 6000;
	static double PHYSICAL_DISTANCE_THRESHOLD = 3;
	Transform prev = null;
	Transform curr = null;
	int prevMarked = -1;
	int currMarked = -1;
	boolean useKalman=false;

	

	

	public double[] bruteForceICP(int iterations) {
		
		//first method will be applying translation so that the best matching points are at the same spot
		//and then rotating the entire cloud around that spot to see which orientation makes for the best
		//match between layers

		double [] result = new double[3];
		for(int i=0;i<iterations;i++) {
			result = bruteForceSearch(6/Math.pow(Math.sqrt(2), i),8, 3/Math.pow(Math.sqrt(5), i),20, 0.4d,result[0],result[1],result[2]);
		}
		return result;
	}
	
	
	public double[] bruteForceSearch(double Tmul, int Tresolution, double Amul, int Aresolution, double minimalDistance, double xs, double ys,double rs ) {
		
		//for both types of transformations I receive range and resolution
		// resolution - how many points will be examined
		// "Tmul" - multiplied by the resolution to enlarge/shrink the window; 
		//for range=1 - the testing window has the size of resolution × resolution
		//rotations work the same way
		
		double bestScore = -1;
		double[] bestShift = new double[3];

		for(int x=0;x<Tresolution;x++) {
			for(int z=0;z<Tresolution;z++) {
				
				double currentX = ((x-(Tresolution/2))/10d)*Tmul+xs;
				double currentZ = ((z-(Tresolution/2))/10d)*Tmul+ys;
				double starting_angle = Aresolution*Amul;
				
				curr.translate(currentX, currentZ);
				curr.rotate(currentX, currentZ,-starting_angle+rs);
				
				for(int r=-Aresolution;r<=Aresolution;r++) {
					
					curr.rotate(currentX, currentZ, 1*Amul);
					double score = curr.computeMatchingScore(prev,minimalDistance);
			
					if(bestScore<score) {
						bestShift= new double[]{curr.points[0][0],curr.points[0][2],rs+r*Amul};
						bestScore=score;
					}
				}
				curr.resetPositions();
				
			}
			}
		
		
		return bestShift;
	}

	public double[] RANSAC(int loop) {
		
		ArrayList<double[]> transforms = new ArrayList<double[]>();
		double bestScore=0;
		double[] best= new double[3];
		//looking for a, x, z values
		   //pick 3 best matches 
		for(int p=0;p<10;p++) {
			
		copyFeatures();
		double[][] matchedPoints = takeBestMAtches();
		double[] transform = computeTransform(matchedPoints);
		transforms.add(transform);
		curr.translate(transform[0],transform[1]);
		curr.rotate(transform[0], transform[1], transform[2]);
		double score = curr.computeMatchingScore(prev,0.4d);
		curr.resetPositions();
		
		if(bestScore<score) {
			bestScore=score;
			best=transform;
		}
		
		
		
	}
		saveFeaturMatches("FC"+String.valueOf(loop) , featureMatches);
		saveTransforms(String.valueOf(loop) , transforms);
		return best;
	}
	
	void saveFeaturMatches(String oridinalName, ArrayList<double[]> transforms) {
	       try {
	            FileWriter writer = new FileWriter("C:\\Users\\Lenovo\\Desktop\\dane\\Evaluation\\"+oridinalName+".csv", true);
	            for(int i = 0; i<transforms.size();i++)
	            {
	            	double[] curr = transforms.get(i); 
	            	writer.write((curr[1]-curr[4])+";"+(curr[2]-curr[5])+";1");
	            	writer.write("\r\n");  
	            }
	            
	            
	            writer.close();
	        } catch (IOException e) {
	            e.printStackTrace();
	        }
	}
	
	void saveTransforms(String oridinalName, ArrayList<double[]> transforms) {
	       try {
	            FileWriter writer = new FileWriter("C:\\Users\\Lenovo\\Desktop\\dane\\Evaluation\\"+oridinalName+".csv", true);
	            for(int i = 0; i<transforms.size();i++)
	            {
	            	double[] curr = transforms.get(i); 
	            	writer.write(curr[0]+";"+curr[1]+";"+curr[2]);
	            	writer.write("\r\n");  
	            }
	            
	            
	            writer.close();
	        } catch (IOException e) {
	            e.printStackTrace();
	        }
	}
	
	
	void copyFeatures() {
		fetureCopied = new ArrayList<double[]>();
		
		for(int i=0;i<featureMatches.size();i++) {
			double [] c = featureMatches.get(i);
			fetureCopied.add(new double[] {c[0],c[1],c[2],c[3],c[4],c[5],c[6],});
		}
	}
	
	double[] computeTransform(double[][] m) {
		
		Point Ap = new Point(m[0][1],m[0][2]);
		Point A = new Point(m[0][4],m[0][5]);
		
		Point Bp = new Point(m[1][1],m[1][2]);
		Point B = new Point(m[1][4],m[1][5]);
		
		Point Cp = new Point(m[2][1],m[2][2]);
		Point C = new Point(m[2][4],m[2][5]);
		//constructing equation 1 from match 1 and 3
		double equals1 = Cp.y-Ap.y;
		double sin1 = -C.y-(-A.y);
		double cos1 = C.x-A.x;
		//constructing equation 1 from match 1 and 3
		double equals2 = Cp.y-Bp.y;
		double sin2 = -C.y-(-B.y);
		double cos2 = C.x-B.x;

		double[] histogram1 = solveTrygonometricEquation(equals1,sin1,cos1);
		double[] histogram2 = solveTrygonometricEquation(equals2,sin2,cos2);
		
		double smallest=10;
		double smallestIndex=-300;
		for(int i=-100;i<101;i++) {
			double value = histogram1[i+100]+histogram2[i+100];
		if(value<smallest) {
			smallestIndex=i;
			smallest=value;
		}
			
		}
		double rotation = (Math.PI*(double)smallestIndex)/100d;
		
		double h1=Ap.x-(A.x*Math.cos(rotation)-A.y*Math.sin(rotation));
		double h2=Bp.x-(B.x*Math.cos(rotation)-B.y*Math.sin(rotation));
		double h3=Cp.x-(C.x*Math.cos(rotation)-C.y*Math.sin(rotation));
		
		double k1=Ap.y-(A.x*Math.sin(rotation)+A.y*Math.cos(rotation));
		double k2=Bp.y-(B.x*Math.sin(rotation)+B.y*Math.cos(rotation));
		double k3=Cp.y-(C.x*Math.sin(rotation)+C.y*Math.cos(rotation));
		
		return new double[]{h1,k1,smallestIndex};
	}
	
	double[] solveTrygonometricEquation(double equals, double sin, double cos) {
		
		double[] histogram = new double[201];
		
		for(int i=-100;i<101;i++) {
			double angle = (Math.PI*(double)i)/100d;
			double value = sin*Math.sin(angle)+cos*Math.cos(angle);
			double distance = Math.abs(value-equals);
			histogram[i+100]=distance;
		}
		
		return histogram;
	}
	
	
	
	double[][] takeBestMAtches(){
		double[][] pickedMatches = new double[3][7];
		
		double first=0,second=0,third=0;
		int firstIndex=0,secondIndex=0,thirdIndex=0;
		
		for(int i=0;i<fetureCopied.size();i++) {
			double d = fetureCopied.get(i)[6];
			
			if(d>first) {firstIndex=i;first=d;}
			else if(d>second) {secondIndex=i;second=d;}
			else if(d>third) {thirdIndex=i;third=d;}
			
		}
		
		pickedMatches[0]=fetureCopied.get(firstIndex);
		pickedMatches[1]=fetureCopied.get(secondIndex);
		pickedMatches[2]=fetureCopied.get(thirdIndex);
		
		featureMatches.remove(firstIndex);
		
		return pickedMatches;
	}
	
	
	
	private double InferRotation(Point A, Point B) {
		//Point A acts as a pivot
		double yShift = (B.x-A.x);
		double xShift = (B.y-A.y);
		double angle = Math.toDegrees(Math.atan(yShift==0?(xShift*99999999):(xShift/yShift)));
	    
	    double result=0;
	    if(xShift<=0) {
	    	if(yShift<0) {result=90-angle;}
	    	else {result=270-angle;}}
	    else {
	    	if(yShift<0) {result=90-angle;}
	    	else {result=270-angle;}}
		
		
		
		return result;
	}
	
	public SLAM(FeatureCloud t_minus_1, FeatureCloud t) {
		
		prev = new Transform(t_minus_1);
		curr = new Transform(t);
		//now we're matching the features
		this.featureMatches = new ArrayList<double[]>();
		match_features(t_minus_1,t);
		
		
	}
	
	public void match_features(FeatureCloud Previous , FeatureCloud Current ) {
		
		for(int fl=0;fl<Previous.features.size();fl++) {
		double[] vectorL = Previous.features.get(fl);
		double Lx = Previous.featurePositions.get(fl)[0];
		double Ly = Previous.featurePositions.get(fl)[1];
		double Rx = 0;
		double Ry =0;
		
		double smallest =999999999;
		int smallest_index=1;
		
		for(int fr=0;fr<Current.features.size();fr++) {
			
			double[] vectorR = Current.features.get(fr);
			Rx = Current.featurePositions.get(fr)[0];
			Ry = Current.featurePositions.get(fr)[1];
			
			
				double distance =0;
				for(int v =0;v<128;v++) {
					distance+=Math.abs(vectorR[v]-vectorL[v]);
				}
				
				if(distance<smallest) {
					smallest=distance;
					smallest_index=fr;
				}	
			
		}
		
		if(smallest<PROXIMITY_THRESHOLD) {
			double dx = Previous.featurePositions.get(fl)[0]-Current.featurePositions.get(smallest_index)[0];
			double dz = Previous.featurePositions.get(fl)[2]-Current.featurePositions.get(smallest_index)[2];
			double trueDistance = Math.sqrt(dx*dx+dz*dz);
			if(trueDistance<PHYSICAL_DISTANCE_THRESHOLD)
			featureMatches.add(new double[] {
					fl,Previous.featurePositions.get(fl)[0],Previous.featurePositions.get(fl)[2],
					smallest_index,Current.featurePositions.get(smallest_index)[0],Current.featurePositions.get(smallest_index)[2],
					smallest});
			
			
		}
		
		
	}
		
		
		
		
	}
}

class Transform{
	double[][] points;
	final double[][] pointsFinal;
	
	//computes mass center of points within radius
	public double[] computeCenter(double radius) {
		
		double xc =0;
		double zc =0;
		double c =0;
		
		for(int i=0;i<this.points.length;i++) {
			double x = this.points[i][0];
			double z = this.points[i][2];
			//distance from 0,0
			double distance = Math.sqrt((x*x)+(z*z));
			if(distance<=radius) {
				c++;
				xc+=x;
				zc+=z;
			}
			
		}
				
		
		return new double[] {xc/c,zc/c}; 
	}
	
	public void resetPositions() {
		
		for(int i=0;i<points.length;i++) {
			points[i]=pointsFinal[i];
		}
		
	}
	
	public double computeMatchingScore(Transform another, double MINIMAL_DISTANCE) {
		
		//the comparing strategy is this: we're counting points from group B (prev) that are in close proximity
		// to any of the points in group A (curr)
		double count=0;
		
		for(int i=0;i<another.points.length;i++) {
			
			boolean hasMatch = false;
			double anotherX = another.points[i][0];
			double anotherZ = another.points[i][2];
			
			for(int u=0;u<this.points.length;u++) {
				double xShiftSquared = Math.pow(this.points[u][0]-anotherX,2);
				double zShiftSquared = Math.pow(this.points[u][2]-anotherZ,2);
				double distance = Math.sqrt(xShiftSquared+zShiftSquared);
				
				
				if(distance<=MINIMAL_DISTANCE) {count+=Math.pow(MINIMAL_DISTANCE-distance,2);}
			}
			
		}
		
		return count;
	}
	
	public void translate(double x, double z) {
		

		for(int i =0;i<points.length;i++) {
			double currX = points[i][0];
			double currY = points[i][1];
			double currZ = points[i][2];
			
			points[i]=new double[] {currX+x,currY,currZ+z};
			
		}
		
	}
	
	public void rotate(double pivotX, double pivotZ, double rotation){
		
	
		
		for(int i =0;i<points.length;i++) {

			double xShift = (points[i][0]-pivotX);
			double yShift = (points[i][2]-pivotZ);
			double angle = Math.toDegrees(Math.atan(yShift==0?(xShift*99999999):(xShift/yShift)));
		    
		    double result=0;
		    if(xShift<=0) {
		    	if(yShift<0) {result=90-angle;}
		    	else {result=270-angle;}}
		    else {
		    	if(yShift<0) {result=90-angle;}
		    	else {result=270-angle;}}
		    
		    double radians = ((-rotation+result)/360d)*Math.PI*2d;
		    double distance = Math.sqrt(Math.pow(xShift, 2)+Math.pow(yShift, 2));
		    double cz = pivotZ-Math.sin(radians)*distance;
		    double cx = pivotX-Math.cos(radians)*distance;
		    double cy = points[i][1]; 
			
			points[i]=new double[] {cx,cy,cz};
			
		}
		
		
	} 
	
	public Transform(FeatureCloud FC) {
		pointsFinal = new double[FC.featurePositions.size()+1][3];
		points = new double[FC.featurePositions.size()+1][3];
		points[0]=new double[] {0d,0d,0d};
		pointsFinal[0]=new double[] {0d,0d,0d};
		for(int i=1;i<FC.featurePositions.size()+1;i++) {
			points[i]=FC.featurePositions.get(i-1);
			pointsFinal[i]=FC.featurePositions.get(i-1);
		}
	}
	
	public Transform(double[][] FC) {
		pointsFinal = new double[FC.length+1][3];
		points = new double[FC.length+1][3];
		points[0]=new double[] {0d,0d,0d};
		pointsFinal[0]=new double[] {0d,0d,0d};
		for(int i=1;i<FC.length+1;i++) {
			points[i]=FC[i-1];
			pointsFinal[i]=FC[i-1];
		}
	}
}