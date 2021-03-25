import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

//this class matches features from stereo vision and outputs best matches with 3d positions
public class FeatureCloud {
	ArrayList<double[]> features=null;
	ArrayList<double[]> featurePositions=null;
	static double PROXIMITY_THRESHOLD = 5000;
	
	public FeatureCloud(String path) {
		this.features = new ArrayList<double[]>();
		this.featurePositions = new ArrayList<double[]>();
		
			try {
				FileReader reader = new FileReader(path);
				BufferedReader bufferedReader = new BufferedReader(reader);
				
				String line;
				
				while ((line = bufferedReader.readLine()) != null) {
				
					
					String[] doubles = line.split(";");
					double[] newVector = new double[128];
					if(doubles.length==131){
					for(int i=0;i<128;i++) {newVector[i]=Double.parseDouble(doubles[i]);}
					
					double x = Double.parseDouble(doubles[128]);
					double y = Double.parseDouble(doubles[129]);
					double z = Double.parseDouble(doubles[130]);
					
					this.features.add(newVector);
					this.featurePositions.add(new double[] {x,y,z});
				}
					
				}
				reader.close();
			} catch (IOException e) {
			e.printStackTrace();
			}


	}
	
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
				//featurePositions.add(new double[] {Lx,Ly,t[1],t[2]});
				featurePositions.add(t);
				
				
					//System.out.println(Lx+", "+Ly+", "+Rx+", "+Ry);
				
				
			}
			
			
		}
		
	}

	
	double[] triangulate(double lx, double ly, double rx, double ry) {
		
		
		//converting the pixel positions to sensor dimensions
		double wlx = lx*0.1;
		double wrx = rx*0.1;
		
		//setting the center in the middle of the sensor
		double x1 = wlx-96d;
		double x2 = wrx-96d;
		
		//computing match coordinates
		double z = (2*45.31138d)/(x1-x2);
		double x = x1*z/45.31138d;
		double y = 1*z/45.31138d;
		

		return new double[] {x,y,z};
	}
}
