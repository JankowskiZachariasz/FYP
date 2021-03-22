import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class Simulator {
	
	double[][] map;
	Transform mapTransform = null;
	boolean absolutePositions = false;//true = absolutePositions, false = relative to the starting points
	boolean camView = false; //if true map doesn't require any transformations
	boolean photogrammetryPoints = false;
	ANNdata ann=null;
	ArrayList<ANNdata> trajectories = null;
	double[][] vertices = null;
	ArrayList<Triangle> triangles; 
	
	
	//loop-related variables
	int t =0;
	int s =0;
	
	public Simulator(boolean absolute,boolean camView, boolean photogrammetryPoints) {
		this.absolutePositions = absolute;
		this.camView = camView;
		this.photogrammetryPoints = photogrammetryPoints;
	}
	
	public void simulate(){
		
		for(int t=0;t<trajectories.size();t++) {
			
			ANNdata current = trajectories.get(t);
			this.ann = current;
//			Main.ann = this.ann;
			//let's determine number of steps
			int steps = current.shifts.size();
			
			//now we either leave the positions as they are (absolute by default) or we set it to 0s
			
			double x = current.positionX;
			double z = current.positionY;
			double r = current.rotation;
			
			if(!absolutePositions)
			{
				current.positionX=0d;
				current.positionY=0d;
				current.rotation=0d;
			}
			
			for(int s=0;s<steps;s++) {
				
				mapTransform.resetPositions();
				//mapTransform.
				double[] change = current.shifts.get(s);
				x+=change[0];
				z+=change[1];
				r+=change[2];
				mapTransform.translate(z, -x);
				mapTransform.rotate(0, 0, -r);
				
				
				current.generateAnnStep(mapTransform,s+1,photogrammetryPoints);
				
				try {TimeUnit.SECONDS.sleep(1);}catch(Exception exc) {}
				
				
			}
			
		}
		
	}
	
	public void simulateOneStep(){
	
		if(this.t<trajectories.size()){
			
			ANNdata current = trajectories.get(t);
			this.ann = current;
//			Main.ann = this.ann;
			//let's determine number of steps
			int steps = current.shifts.size();
			
			//now we either leave the positions as they are (absolute by default) or we set it to 0s
			
			double x = current.positionX;
			double z = current.positionY;
			double r = current.rotation;
			
			if(!absolutePositions)
			{
				current.positionX=0d;
				current.positionY=0d;
				current.rotation=0d;
			}
			
			if(this.s<steps) {
				
				mapTransform.resetPositions();
				//mapTransform.
				double[] change = current.shifts.get(s);
				x+=change[0];
				z+=change[1];
				r+=change[2];
				if(!camView) {
					mapTransform.translate(z, -x);
					mapTransform.rotate(0, 0, -r);
				}
				
				
				
				current.generateAnnStep(mapTransform,s+1,photogrammetryPoints);
				
				//try {TimeUnit.SECONDS.sleep(1);}catch(Exception exc) {}
				
				this.s++;
			}
			else {this.t++; this.s=0;}
			
		}
		
	}
	
	public void setMapVerticies(double[][] vertices) {
		//vertices -> Line segments -> points
		
		ArrayList<double[]> points = new ArrayList<double[]>();
		
		//number of vertices in a polygon = number of sides in that polygon
		for(int i = 0; i<vertices.length;i++) {//i determines point A
			int iB =0;
			if(i<vertices.length-1) iB=i+1;
			
			double[] A = vertices[i];
			double[] B = vertices[iB];
			
			
			Segment segment = new Segment(A[0],A[1],B[0],B[1]);
			ArrayList<double[]> segmentPoints = segment.points();
			for(int sp=0;sp<segmentPoints.size();sp++)
				points.add(segmentPoints.get(sp));
			
		}
		
		saveMap(points);
		
		ann = new ANNdata();
		this.vertices = vertices;
		
	}
	
	public void setMapPoints(ArrayList<double[]> points, double[][] vertices) {
		
		
		ArrayList<double[]> rewrite = new ArrayList<double[]>();
		
		for(int i=0;i<points.size();i++) {
			double[] curr = points.get(i);
			rewrite.add(new double[] {curr[0],curr[2]});
		}
				
		
		this.vertices = vertices;
		saveMap(rewrite);
		
	}
	
	public void setMapFromTriangles(ArrayList<Triangle> tri) {
		
		
		ArrayList<double[]> rewrite = new ArrayList<double[]>();
		
		for(int i=0;i<tri.size();i++) {
			
			rewrite.add(new double[] { tri.get(i).vertices[0],tri.get(i).vertices[1]});
			rewrite.add(new double[] { tri.get(i).vertices[2],tri.get(i).vertices[3]});
			rewrite.add(new double[] { tri.get(i).vertices[4],tri.get(i).vertices[5]});
			
		}
				
		saveMap(rewrite);
		
	}
	
	public void setMapRandom() {}
	
	public void setTrajectories(String[] path){
		
		trajectories = new ArrayList<ANNdata>();
		
		for(int i=0; i<path.length;i++) {
			ANNdata annData=new ANNdata(path[i]);
			trajectories.add(annData);
		}
		
		this.ann = trajectories.get(0);
//		Main.ann = this.ann;
		
	}
	
	
	public String generateRandomTrajectory(int points){
		
		//break map vertices into triangles to see if a random point is inside the polygon
		generateTriangles();
		//we are generating n random points, checking if there are no obstacles between consecutive ones, simulating rotation
		//(aligning it with the new path in 4 steps)
		double xMin =0,xMax=0,yMin=0,yMax=0;
		
		for(int i =0;i<vertices.length;i++) {
			double x = vertices[i][0];
			double y = vertices[i][1];
			if(x>xMax)xMax=x;
			if(x<xMin)xMin=x;
			if(y>yMax)yMax=y;
			if(y<yMin)yMin=y;
		}
		
		ArrayList<double[]> turningPoints = new ArrayList<double[]>();
		
		
		for(int k=0;k<points;k++) {
			
			//System.out.println(k);
			double drawX = xMin + Math.random()*(xMax-xMin);
			double drawY = yMin + Math.random()*(yMax-yMin);
			if(k!=0) {
				Point previous = new Point(turningPoints.get(k-1)[0],turningPoints.get(k-1)[1]);
				Point current = new Point(drawX,drawY);
				
				if(containedInside(drawX,drawY)&&canBeTraversed(new Line(previous,current))&&isFarFromTheWalls(current)) {	
					turningPoints.add(new double[] {drawX,drawY});
			}
			else {k--; 
			} 
			}
			else {
				drawX = xMin + Math.random()*(xMax-xMin);
				drawY = yMin + Math.random()*(yMax-yMin);
				if(containedInside(drawX,drawY)&&isFarFromTheWalls(new Point(drawX,drawY)))
					turningPoints.add(new double[] {drawX,drawY});	
				else k--;
			}
			
		}
		
		double[] initialPos = turningPoints.get(0);
		double rotation =0;
		String trajectory=(initialPos[0]+",3,"+initialPos[1]+",0,"+rotation+",0,0"+"\r\n");
		for(int t=1;t<turningPoints.size();t++) {
			double[] p = turningPoints.get(t-1);
			double[] n = turningPoints.get(t);
			rotation=InferRotation(new Point(p[0],p[1]),new Point(n[0],n[1]));
			trajectory+=(p[0]+",3,"+p[1]+",0,"+rotation+",0,20"+"\r\n");
			trajectory+=(n[0]+",3,"+n[1]+",0,"+rotation+",0,12"+"\r\n");

		}
		
		
		
		

		
	return trajectory;
		
		
		
		
		
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
		
		result+=180;
		
		return result%360;
	}
	
	private boolean isFarFromTheWalls(Point q){
		
		for(int i =0;i<map.length;i++) {
			double[] p = map[i];
			double distance = Math.sqrt(Math.pow(p[0]-q.x, 2)+Math.pow(p[2]-q.y, 2));
			if(distance<2d)return false;
		}
		
		return true;
	}
	
	private boolean canBeTraversed(Line line) {
		//draw a line between the two points and check if any of the map points lays on (or is close enough to) the line
		for(int i =0;i<map.length;i++) {
			double[] p = map[i];
			if(line.DoesColide(p[0],p[2]))return false;
		}
		return true;
	}
	
	private boolean containedInside (double x, double y){
		
		
		for(int i=0;i<triangles.size();i++) {
		if(triangles.get(i).contains(x, y))return true;	
		}
		return false;
	}
	
	private void generateTriangles() {
		
		ArrayList<double[]> eachVertex = new ArrayList<double[]>();
		for(int i =0;i<vertices.length;i++) 
			eachVertex.add(new double[]{Double.valueOf(vertices[i][0]),Double.valueOf(vertices[i][1])});
		
		triangles = new ArrayList<Triangle>();
		int i =0;
		while(eachVertex.size()>2) {
			
			double[] A = eachVertex.get(i==0?(eachVertex.size()-1):(i-1));
			double[] B = eachVertex.get(i);
			double[] C = eachVertex.get(i==eachVertex.size()-1?(0):(i+1));
			
			Triangle triangle = new Triangle(new double[] {A[0],A[1],B[0],B[1],C[0],C[1]});
			if(triangle.isConvex) {
				triangles.add(triangle);
				eachVertex.remove(i);
			}
			else i++;
		if(i>=eachVertex.size()) i=0;
		}
		
//		for(int y =0;y<triangles.size();y++) {
//			Triangle t = triangles.get(y);
//			for(int n =0;n<6;n++)
//			System.out.print(t.vertices[n]+", ");
//			System.out.println();
//		}
	}
	
	private void saveMap(ArrayList<double[]> points) {
		this.map = new double[points.size()][2];
		
		for(int sp=0;sp<points.size();sp++) {
			double x = points.get(sp)[0];
			double z = points.get(sp)[1];
			this.map[sp]=new double[] {x,0,z};
		}
		
		mapTransform = new Transform(this.map);
		
	}
}

class Triangle{
	boolean isConvex=false;
	double[] vertices=null; 
	
	public Triangle(double[] vertices) {
		this.vertices = vertices;
		//vertices = [xA,yA,xB,yB,xC,yC]
		//checking on which side of the line created by first 2 points lays the 3rd point
		double dy = vertices[3]-vertices[1];
		double dx = vertices[2]-vertices[0];
		double a=dy/dx;
		double b=vertices[1]-vertices[0]*a;
		//assuming counter clockwise traversal of the polygon thus the 3rd point
		//has to be above the extended line created by first 2 points if Bx>Ax and vice versa
		//boolean compare = vertices[4]*
		this.isConvex = ((vertices[2]>vertices[0])==(vertices[5]>(vertices[4]*a+b)));
		
	}
	
	boolean contains(double x, double y) {
		//getting the 3 lines constituting the triangle and checking if the given point is on the same side as the 3rd vertex 
		Point A = new Point(vertices[0],vertices[1]);
		Point B = new Point(vertices[2],vertices[3]);
		Point C = new Point(vertices[4],vertices[5]);
		
		Line AB = new Line(A,B);
		Line BC = new Line(B,C);
		Line AC = new Line(A,C);
		
		
		return ((A.y>A.x*BC.a+BC.b==y>x*BC.a+BC.b)&&
				(B.y>B.x*AC.a+AC.b==y>x*AC.a+AC.b)&&
				(C.y>C.x*AB.a+AB.b==y>x*AB.a+AB.b));
		
	}
	

	
}

class Point{
	double x;
	double y;
	
	public Point(double x, double y) {
		this.x=x;
		this.y=y;
		
	}
	
	public void print() {
		System.out.println("x:"+x+", y:"+y);
	}
}

class Line {
	
	Point A,B;
	double xMin=9999999,xMax=0;
	double a;
	double b;
	public Line(Point A, Point B) {
		this.A=A;
		this.B=B;
		double dy = B.y-A.y;
		double dx = B.x-A.x;
		this.a=dy/dx;
		this.b=A.y-A.x*a;
		
		if(A.x>xMax)xMax=A.x;
		if(A.x<xMin)xMin=A.x;
		if(B.x>xMax)xMax=B.x;
		if(B.x<xMin)xMin=B.x;
	}
	
	public boolean DoesColide(double x, double y) {
		
		if(x>xMin&&x<xMax) {
			double distance = Math.abs(a*x-y+b)/Math.sqrt((a*a)+1);

			if(distance<4d) {
				
				
//				System.out.println(distance);
//				new Point(x,y).print();
//				A.print();B.print();
//				System.out.println(xMin+", "+xMax);
				return true;
			}
			else return false;
		}
		else return false;
		
	}
	
	
}

class Segment{
	
	double xInterval, a, b, xA, yA, xB, yB;
	double DISTANCE_BETWEEN_CONSECUTIVE_POINTS = 0.1d;
	
	public Segment( double xA,double yA,double xB,double yB) {
		
		this.xA = xA;
		this.yA = yA;
		this.xB = xB;
		this.yB = yB;
		
		
		double dy = yB-yA;
		double dx = xB-xA;
		this.a=dy/dx;
		this.b=yA-xA*this.a;
		
		//xInterval for points probing should depend on the line slope
		double slope = Math.atan(this.a);
		this.xInterval = Math.cos(slope)*DISTANCE_BETWEEN_CONSECUTIVE_POINTS;
		if(xA>xB)this.xInterval=-this.xInterval;
		
	}
	
	public ArrayList<double[]> points(){
		ArrayList<double[]> generatedPoints = new ArrayList<double[]>();
		int numberOfBetweenPoints = (int)Math.floor(Math.abs(xA-xB)/Math.abs(xInterval));
		
		generatedPoints.add(new double[] {xA, yA});
		double x = xA;
		for(int p=0;p<numberOfBetweenPoints;p++) {
			x+=xInterval;
			double y = a*x+b;
			generatedPoints.add(new double[] {x, y});
		}
		generatedPoints.add(new double[] {xB, yB});
		return generatedPoints;
	}
}
