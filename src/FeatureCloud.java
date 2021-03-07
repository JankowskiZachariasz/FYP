import java.util.ArrayList;

//this class matches features from stereo vision and outputs best matches with 3d positions
public class FeatureCloud {
	ArrayList<double[]> features=null;
	ArrayList<double[]> featurePositions=null;
	static double PROXIMITY_THRESHOLD = 5000;
	
	public FeatureCloud(ImageSIFT Left, ImageSIFT Right) {
		
		this.features = new ArrayList<double[]>();
		this.featurePositions = new ArrayList<double[]>();
		
		for(int fl=0;fl<Left.features.size();fl++) {
			double[] vectorL = Left.features.get(fl);
			double Lx = Left.featurePositions.get(fl)[0];
			double Ly = Left.featurePositions.get(fl)[1];
			double Rx = 0;
			double Ry =0;
			
			double smallest =999999999;
			int smallest_index=1;
			
			for(int fr=0;fr<Right.features.size();fr++) {
			
				//taking advantage of two facts: 
				//1)features must lay on the same y coordinate, 
				//2) left camera x position sets the limit for where we can find the same feature on the right photo
				
				double[] vectorR = Right.features.get(fr);
				Rx = Right.featurePositions.get(fr)[0];
				Ry = Right.featurePositions.get(fr)[1];
				
				
				if(Rx<Lx&&Math.abs(Ly-Ry)<3) {
					double distance =0;
					for(int v =0;v<128;v++) {
						distance+=Math.abs(vectorR[v]-vectorL[v]);
					}
					
					if(distance<smallest) {
						smallest=distance;
						smallest_index=fr;
					}	
				}	
			}
			
			//time to triangulate 
			if(smallest<PROXIMITY_THRESHOLD) {
				Rx = Right.featurePositions.get(smallest_index)[0];
				Ry = Right.featurePositions.get(smallest_index)[1];
				features.add(vectorL);
				double[] t = triangulate(Lx,Ly,Rx,Ry);
				//if(t[1]>-100&&t[1]<100)
				featurePositions.add(new double[] {Lx,Ly,t[1],t[2]});
				
				
					//System.out.println(Lx+", "+Ly+", "+Rx+", "+Ry);
				
				
			}
			
			
		}
		
	}

	
	double[] triangulate(double lx, double ly, double rx, double ry) {
		
		double wly = ly*0.047388d/2;
		double wlx = lx*0.056375d/2;
		double wry = ry*0.047388d/2;
		double wrx = rx*0.056375d/2;
		
		double x1 = wlx-22.74d;
		double x2 = wrx-22.74d;
		double y1 = 12.795d-wly;
		double y2 = wry-12.795d;
		
		
		
//		double x = (lx-480)*vectotMultiplier;
//		double y = (ly-270)*vectotMultiplier;
//		double z = (603.56)*vectotMultiplier;
//		Z = ( b * f ) / ( x1 - x2 )
		double z = (2*14.3d)/Math.abs(x1-x2);
//		X = x1 * Z / f
		double	x= x1*z/14.3d;
//		Y = y1 * Z / f
		double y = y1*z/14.3d;
		
		//double z = 14.3d*p;
		
		System.out.println("UWAGA!, Z:"+z);
		return new double[] {x,y,z};
	}
}
