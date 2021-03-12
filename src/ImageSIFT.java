import java.awt.Color;
import java.awt.geom.AffineTransform;
import java.awt.image.AffineTransformOp;
import java.awt.image.BufferedImage;
import java.awt.image.ColorModel;
import java.awt.image.WritableRaster;
import java.util.ArrayList;
import Jama.Matrix;


//zrobi³em dwie zmiany (wielkosc obrazu)(ma byæ 0) oraz intesity threshold (ma byc 0.6)
public class ImageSIFT {
	
	//sift parameters
	static double KEYPOINT_INTENSITY_THRESHOLD = 0.600d;
	static double KAPPA_EDGES = 0.15d;
	static double THRESHOLD_EDGES = -1000000;
	
	Octave[] octaves=null;
	ArrayList<double[]> features=null;
	ArrayList<double[]> featurePositions=null;
	
	long[][] original=null;
	

	public ImageSIFT(BufferedImage img, int octaveCount, int[][][][] guassians){
		
		this.features = new ArrayList<double[]>();
		this.featurePositions = new ArrayList<double[]>();
		this.octaves = new Octave[5];
		
		for(int i=0;i<octaveCount;i++) {
			this.octaves[i] = new Octave(img, 5, "1",(int)Math.pow(2,0+i));
			if(i==0) {this.original=this.octaves[i].original;}
			this.octaves[i].generate_guassians(guassians[0]);
			this.octaves[i].generateDOGs();
			this.octaves[i].findKeypoints();
			this.octaves[i].computeHarrisdeterminant();
			this.octaves[i].discardEdges();
			try{this.octaves[i].subPixelPositions();}catch(Exception e){System.out.println("Failed to optimise keypoint position");}
			this.octaves[i].DescribeKeypoints(guassians[1]);
			
		}
		
		//translating all discovered features into the coordinate system of the biggest photo
		for(int i=0;i<octaveCount;i++) {
			ArrayList<double[]> currFetures = this.octaves[i].fetures;
			ArrayList<point> currPositions = this.octaves[i].feturePositions;
			
			for(int c =0;c<currFetures.size();c++) {
				features.add(currFetures.get(c));
				double x = currPositions.get(c).x*Math.pow(2,i);
				double y = currPositions.get(c).y*Math.pow(2,i);
				featurePositions.add(new double[] {x,y});//just x and y
			}
			
		}

		System.out.println("finished processing photo");
		
	}
	
	static int[][][] initialiseGuassianLibrary(int kernelsize, int numscales, double initSigma, double SigmaMultiplier) {
		int[][][] guassianLibrary = new int[numscales][kernelsize][kernelsize];
		
		for(int i = 0;i<numscales;i++) {
			guassianLibrary[i]=generateGuassian(kernelsize,initSigma*Math.pow(SigmaMultiplier, i),100);
		}
		
		
		return guassianLibrary;
	}
	
	static int[][][] initialiseCircularGuassian(int kernelsize, int numscales, double initSigma, double SigmaMultiplier) {
		int[][][] guassianLibrary = new int[numscales][kernelsize][kernelsize];
		
		for(int i = 0;i<numscales;i++) {
			guassianLibrary[i]=generateGuassian(kernelsize,initSigma*Math.pow(SigmaMultiplier, i),100);
		}
		
		
		return guassianLibrary;
	}
	
	static double[][] normalize(long[][] input){
        long highest = 0;
        long lowest = 0;
        double[][] output = new double[input.length][input[0].length] ;
        for( int x = 0; x < input.length-10; x++ )
	        for( int y = 0; y < input[0].length-10; y++ ) {
	        	
	        	long value = input[x][y];
	        	if (value>highest)highest=value;
	        	if (value<lowest)lowest=value;	
	        }
        
        
        long fullRange = Math.abs(highest-lowest);

	        	//long value = (long)((octave4.DOG[1][x][y]-lowest)/(double)fullRange*255d);
        
        for( int x = 0; x < input.length-10; x++ )
	        for( int y = 0; y < input[0].length-10; y++ ) {
	        	

	        	output[x][y] = ((input[x][y]-lowest)/(double)fullRange);
	        }
        return output;
	}
	
	static int[][] generateGuassian(int n, double sigma, int grid) {
		double mean=n*0.5d;
		
		//take 1000 points distributed evenly on your grid *(x by x) and 
		
		Tile[][] tiles = new Tile[n][n];
		for(int y=0;y<n;y++)
		for(int x=0;x<n;x++){
			tiles[x][y]= new Tile(x,y);
		}
		for(int i =0;i<grid;i++)
		for(int j =0;j<grid;j++) {
			double xx=((double)n/(grid*2))+i*(double)n/grid;
			double yy=((double)n/(grid*2))+j*(double)n/grid;
			int x = (int)(Math.floor(xx));
			int y = (int)(Math.floor(yy));
			tiles[x][y].add(
			 Math.exp( -0.5 * (Math.pow((xx-mean)/sigma, 2.0) + Math.pow((yy-mean)/sigma,2.0)) )
             / (2 * Math.PI * sigma * sigma)
			);
		}
		
				
		double multiplier = 1/tiles[0][0].value();
		int[][] values = new int[n][n];
		for(int y=0;y<n;y++)
			for(int x=0;x<n;x++){
				values[x][y] = (int)Math.round(tiles[x][y].value()*multiplier);
			}
		
		
		return values;
	}
	
	static class Tile{
		private double sum =0;
		private int n=0;
		int x,y;
		public Tile(int x, int y)
		{this.x=x;this.y=y;}
		
		public void add(double value ) {
			sum+=value;
			n++;
		}
		public double value() {
			if(n!=0)
			return sum/n;
			else return 0;
		}
	}
	
	static BufferedImage deepCopy(BufferedImage bi) {
		 ColorModel cm = bi.getColorModel();
		 boolean isAlphaPremultiplied = cm.isAlphaPremultiplied();
		 WritableRaster raster = bi.copyData(null);
		 return new BufferedImage(cm, raster, isAlphaPremultiplied, null);
		}

	
	class point{
		float x,y;
		int scalespace;
		public point(int x,int y, int scalespace) {
			this.x=x;this.y=y;
			this.scalespace = scalespace;
		}
		
	}

	class Feature{
		double Baserotation, SecondBaserotation, x,y;
		Octave o; 
		boolean second = false;
		int scalespace;
		
		public Feature(point p, Octave o, int[][] guassian) {
			this.x=p.x;
			this.y=p.y;
			this.o=o;
			
			int sideLength = 16;
			int xOrigin = (int)x-8;
			int yOrigin = (int)y-8;
			
			
			double[] histogram = new double[36];
			for(int f=0;f<36;f++)histogram[f]=0;
			
			
			int ss = p.scalespace;
			this.scalespace =  p.scalespace;
			for(int xx=0;xx<sideLength;xx++)
				for(int yy=0;yy<sideLength;yy++) {
					int posX=xOrigin+xx;
					int posY=yOrigin+yy;
					
					if((int)posX>0&&
					   (int)posY>0&&
					   (int)posX<o.w-1&&
					   (int)posY<o.h-1) {
						
						double xp1 = o.guassianised[ss][(int)posX+1][(int)posY];
						double xm1 = o.guassianised[ss][(int)posX-1][(int)posY];
						double yp1 = o.guassianised[ss][(int)posX][(int)posY+1];
						double ym1 = o.guassianised[ss][(int)posX][(int)posY-1];
						
						double gradientMagnitude = Math.sqrt(Math.pow(xp1-xm1, 2)+Math.pow(yp1-ym1, 2));
						double gradientOrientation = Math.toDegrees(Math.atan((yp1-ym1)/(xp1-xm1)));
			
						
					    //double distance = Math.sqrt(Math.pow(posX-x, 2)+Math.pow(posY-y, 2));
						double xShift = (xp1-xm1);
						double yShift = (yp1-ym1);
						double angle = Math.toDegrees(Math.atan(yShift==0?(xShift*99999999):(xShift/yShift)));
					    
					    double result=0;
					    if(xShift<=0) {
					    	if(yShift<0) {result=90-angle;}
					    	else {result=270-angle;}}
					    else {
					    	if(yShift<0) {result=90-angle;}
					    	else {result=270-angle;}}
					    
					    histogram[(int)(result/10d)]+=gradientMagnitude*guassian[xx][yy];
						
						
						

					}
				}
			
			double greatest=0d, second=0d;
			int indexGreatest=-1, indexSecond =-1;
			for(int f=0;f<36;f++) {
				if (histogram[f]>greatest) {
					indexSecond=indexGreatest;
					second=greatest;
					indexGreatest = f;
					greatest=histogram[f];
				}
				else if (histogram[f]>second) {
					indexSecond=f;
					second=histogram[f];
				}
			}
			
			this.Baserotation = indexGreatest*10;
			if(second>=(0.8d*greatest)) {
				this.second=true;
				this.SecondBaserotation = indexSecond*10;
			}
			
			
			
			
			
		}
		
		public double[] describe(int guassianCircle[][], double rotation ) {
			//I am assuming the 16x16 sample array
			double[][][] descriptor = new double [4][4][8];
			int xOrigin = (int)x-8;
			int yOrigin = (int)y-8;
			int ss = scalespace;
			
			for(int dx=0;dx<4;dx++)
				for(int dy=0;dy<4;dy++) {
					int currentDescriptotOriginX = xOrigin+4*dx;
					int currentDescriptotOriginY = yOrigin+4*dy;
					
						for(int px=0;px<4;px++)
							for(int py=0;py<4;py++) {
								int cx = 0;
								int cy = 0;
								
								{
								double xShift = (currentDescriptotOriginX+px-x);
								double yShift = (currentDescriptotOriginY+py-y);
								double angle = Math.toDegrees(Math.atan(yShift==0?(xShift*99999999):(xShift/yShift)));
							    
							    double result=0;
							    if(xShift<=0) {
							    	if(yShift<0) {result=90-angle;}
							    	else {result=270-angle;}}
							    else {
							    	if(yShift<0) {result=90-angle;}
							    	else {result=270-angle;}}
							    
							    double radians = ((rotation+result)/360d)*Math.PI*2d;
							    double distance = Math.sqrt(Math.pow(xShift, 2)+Math.pow(yShift, 2));
							    cy = (int) Math.round(y-Math.sin(radians)*distance);
							    cx = (int)Math.round(x-Math.cos(radians)*distance);
								}
								
								
								
								
								
								
										//now I need to check magnitude of each pixel in the current descriptor and
										if((int)cx>0&&
										   (int)cy>0&&
										   (int)cx<o.w-1&&
										   (int)cy<o.h-1) {
											
											double xp1 = o.guassianised[ss][(int)cx+1][(int)cy];
											double xm1 = o.guassianised[ss][(int)cx-1][(int)cy];
											double yp1 = o.guassianised[ss][(int)cx][(int)cy+1];
											double ym1 = o.guassianised[ss][(int)cx][(int)cy-1];
											
											double gradientMagnitude = Math.sqrt(Math.pow(xp1-xm1, 2)+Math.pow(yp1-ym1, 2));
											double gradientOrientation = Math.toDegrees(Math.atan((yp1-ym1)/(xp1-xm1)));
								
											
										    //double distance = Math.sqrt(Math.pow(posX-x, 2)+Math.pow(posY-y, 2));
											double xShift = (xp1-xm1);
											double yShift = (yp1-ym1);
											double angle = Math.toDegrees(Math.atan(yShift==0?(xShift*99999999):(xShift/yShift)));
										    
										    double result=0;
										    if(xShift<=0) {
										    	if(yShift<0) {result=90-angle;}
										    	else {result=270-angle;}}
										    else {
										    	if(yShift<0) {result=90-angle;}
										    	else {result=270-angle;}}
										    
										    double newRotation = result-rotation;
										    if(newRotation<0)newRotation+=360;
										    
										    int bin1 = (int)Math.floor(((newRotation)%360d)/45d);
										    int bin2 = (int)Math.ceil(((newRotation)%360d)/45d);
										    if(bin2==8)bin2=0;
										    double proportion = ((((newRotation)%360d)/45d)%1d);
										    
										    descriptor[dx][dy][bin1]+=(gradientMagnitude*guassianCircle[currentDescriptotOriginX+px-xOrigin][currentDescriptotOriginY+py-yOrigin]*proportion);
										    descriptor[dx][dy][bin2]+=(gradientMagnitude*guassianCircle[currentDescriptotOriginX+px-xOrigin][currentDescriptotOriginY+py-yOrigin]*(1d-proportion));
										    }
								 
							}
				}
			
			
			double[] vector = new double[128];
			for(int dx=0;dx<4;dx++)
				for(int dy=0;dy<4;dy++)
					for(int v=0;v<8;v++) {
							vector[dx*32+dy*8+v]=descriptor[dx][dy][v];
						}
			return vector;
			
		}
	
	
		
	}
	
	class Octave{
		
		public long[][] original;
		public long[][][] guassianised;
		public long[][][] DOG;
		int NUMBER_OF_GUASSIANS;
		int w,h,size;
		public long[][] harisValues;
		String octave;
		public ArrayList<point> keypoints;
		public ArrayList<double[]> fetures;
		public ArrayList<point> feturePositions;

		public Octave(BufferedImage original, int number_of_guassians, String octave,int downscaled){
			int h = original.getHeight();
			int w = original.getWidth();
			BufferedImage o = new AffineTransformOp(AffineTransform.getScaleInstance(1f/downscaled, 1f/downscaled), AffineTransformOp.TYPE_BICUBIC).filter(original, new BufferedImage(w/downscaled, h/downscaled,BufferedImage.TYPE_BYTE_GRAY ));
			this.h = o.getHeight();
			this.w = o.getWidth();
			this.original = bufferedImageToArray(o);
			this.NUMBER_OF_GUASSIANS = number_of_guassians;
			guassianised = new long[NUMBER_OF_GUASSIANS][w][h];
			DOG = new long[NUMBER_OF_GUASSIANS-1][w][h];

			this.octave = octave;
			this.keypoints = new  ArrayList<point>();
			this.harisValues= new long[w][h];
		}
		
		public void generate_guassians(int[][][] guassianLibrary) {
			for(int r = 0;r<NUMBER_OF_GUASSIANS;r++) {
				long[][] newimg = new long [w][h];
				
				this.size = guassianLibrary[r].length;
				
				//should be precalculated
			    int sum=0;
			    for(int y=0;y<size;y++)
					for(int x=0;x<size;x++){
						sum+=guassianLibrary[r][x][y];
					}
			    //System.out.println(sum);
			    
			    for( int i = 0; i < w-size; i++ )
		        for( int j = 0; j < h-size; j++ ) {
		    	long sumR=0;
		    	for(int xx=0;xx<size;xx++) 
					for(int yy=0;yy<size;yy++) 
					{	
						sumR+=(guassianLibrary[r][xx][yy]*this.original[i+xx][j+yy]);
					}
	        	newimg[i][j]= sumR/sum;
				}
				
				
				
				guassianised[r]=newimg;
				//System.out.println("finished! - "+(r+1)+"/"+NUMBER_OF_GUASSIANS+", Octave:"+this.octave);
			}
		}
		
		public void generateDOGs() {
			for(int i=0;i<NUMBER_OF_GUASSIANS-1;i++) {
				long[][] newimg = new long [w][h];
				
				for( int x = 0; x < w-size; x++ )
			        for( int y = 0; y < h-size; y++ ) {
			        	
			        	long a = guassianised[i][x][y];
			        	long b = guassianised[i+1][x][y];

			        	newimg[x][y]=a-b;	
			        	
			        	
			        	
			        }
				DOG[i]=newimg;
			}
		}
		
		public void findKeypoints() {
			for(int i =1;i<NUMBER_OF_GUASSIANS-2;i++) {//for(int i =1;i<NUMBER_OF_GUASSIANS-2;i++) {
				for( int x = 1; x < w-size-1; x++ )
			        for( int y = 1; y < h-size-1; y++ ) {
			        	double a = DOG[i][x][y];
			        	boolean smallest=true;
			        	boolean biggest=true;
			        	//boolean smaller = a<DOG[i-1].getRGB( x, y);
			        	for(int e=0;e<3;e++) {
			        		for(int ex=0;ex<3;ex++) {
			        			for(int ey=0;ey<3;ey++) {
			        				
			        				if(!(e==1&&ex==1&&ey==1)) {
			        					double compare = DOG[i-1+e][x-1+ex][y-1+ey];
			        					
			        				smallest = smallest&&Double.compare(a,compare)<0;
			        				biggest = biggest&&Double.compare(a,compare)>0;
			        				}
			        				
			        				if(!smallest&&!biggest)break;
			        			}
			        			if(!smallest&&!biggest)break;
			        		}
			        		if(!smallest&&!biggest)break;
			        	}
			        	if(smallest||biggest) {
			        		if(x<w&&x>0&&y<h&&y>0)
			        		keypoints.add(new point(x,y,i));
			        	}
			        	
			        	
			        }
					
			}
		}
		
		public long[][] runKernel(int[][] kernel, long[][] in) {
			long[][] output = new long [w][h];
			
			

			this.size =13;
			
			//should be precalculated
		
		    
		    
		    for( int i = 0; i < w-size; i++ )
	        for( int j = 0; j < h-size; j++ ) {
	    	
	    	long sum=0;
	    	for(int xx=0;xx<3;xx++) 
			for(int yy=0;yy<3;yy++) 
			{
				long color = in[i+xx][j+yy]*kernel[xx][yy];
				sum+=color;
				
			}
	    	
	    	
        	output[i][j]=sum;
//	    	Color c = new Color((float)(sum/9d),(float)(sum/9d),(float)(sum/9d));
//	    	newimg.setRGB(i, j,c.getRGB());	
	        	
			}
		    
	
		    //scale
//		    for( int i = 0; i < w-size; i++ )
//		        for( int j = 0; j < h-size; j++ ) {
//		        	int color = new Color(newimg.getRGB( i,j)).getRed();
//		        	Color f = new Color((int)normalize((Math.round(color/(float)highest)*255)),0,0);
//		        	newimg.setRGB(i, j,f.getRGB());	
//		        	
//		        }
			
			return output;
		}
		
		public long[][] multiply(long[][] bia, long[][] bib) {
			long[][] output = new long[w][h];

		    
		    for( int i = 0; i < w-size; i++ )
	        for( int j = 0; j < h-size; j++ ) {
	        	
	        	long a = bia[i][j];
	        	long b = bib[i][j];
	        	
	        

	        	output[i][j]=a*b;	
	        	
			}
			
			return output;
		}
		
		public long[][] multiplyScalar(long[][] bia, double scalar) {
			long[][] output = new long[w][h];

		    
		    for( int i = 0; i < w-size; i++ )
	        for( int j = 0; j < h-size; j++ ) {
	        	
	        	long a = bia[i][j];

	        	output[i][j]=(long)(a*scalar);
	        	
			}
			
			return output;
		}
		
		public long[][] add(long[][] bia, long[][] bib) {
			long[][] output = new long[w][h];

		    
		    for( int i = 0; i < w-size; i++ )
	        for( int j = 0; j < h-size; j++ ) {
	        	
	        	long a = bia[i][j];
	        	long b = bib[i][j];

	        	output[i][j]=a+b;	
			}
			
			return output;
		}
		
		public long[][] subtract(long[][] bia, long[][] bib) {
			long[][] output = new long[w][h];

		    
		    for( int i = 0; i < w-size; i++ )
	        for( int j = 0; j < h-size; j++ ) {
	        	
	        	long a = bia[i][j];
	        	long b = bib[i][j];

	        	output[i][j]=a-b;	
			}
			
			return output;
		}
		
		public long[][] bufferedImageToArray( BufferedImage in){
			long[][] output = new long[w][h];

		    
		    for( int i = 0; i < w-size; i++ )
	        for( int j = 0; j < h-size; j++ ) {
	        	
	        	output[i][j]=new Color(in.getRGB( i, j)).getRed();
//	        	Color a = new Color(bia.getRGB( i, j));
//	        	Color b = new Color(bib.getRGB( i, j));
//	        	newimg.setRGB(i, j,a.getRGB()+b.getRGB());
	        	
			}
			
			return output;
			
		}
		
		public void subtractHarris(BufferedImage bia, BufferedImage bib) {
		

		    
		    for( int i = 0; i < w-size; i++ )
	        for( int j = 0; j < h-size; j++ ) {
	        	

	        	Color a = new Color(bia.getRGB( i, j));
	        	Color b = new Color(bib.getRGB( i, j));
	        	
			}
			
		}
		
		public void computeHarrisdeterminant  () {
			
			 int[][] sobelX = new int[][] {{-1,0,1},{-2,0,2},{-1,0,1}};
			 int[][] sobelY = new int[][] {{-1,-2,-1},{0,0,0},{1,2,1}};
			    
			 		long[][] grad_x = this.runKernel(sobelY, this.original);//image run down by sobel pionowe linie
			 		long[][] grad_y = this.runKernel(sobelX, this.original);
					
					
					long[][] gradient_xx = this.runKernel(sobelY, grad_x);
			 		long[][] gradient_xy = this.runKernel(sobelX, grad_x);
					long[][] gradient_yx = this.runKernel(sobelY, grad_y);
					long[][] gradient_yy = this.runKernel(sobelX, grad_y);

				 
				    long[][] determinant = subtract(multiply(gradient_xx, gradient_yy), multiply(gradient_xy, gradient_yx));
				    
				    long[][] trace = add(gradient_xx, gradient_yy);
				    
				    long[][] rCoefficient = subtract(determinant,multiplyScalar(multiply(trace, trace),(KAPPA_EDGES)));
				    
				    this.harisValues  = rCoefficient;
		
				    
		}
		
		private double[] getDerivativeVector (point p) throws Exception {
			int x=(int)p.x;
			int y=(int)p.y;
			int s=(int)p.scalespace;
			double dx = (DOG[s][x+1][y]-DOG[s][x-1][y])/2d;
			double dy = (DOG[s][x][y+1]
					-DOG[s][x-1][y-1])/2d;
			double ds = (DOG[s+1][x][y]-DOG[s-1][x-1][y])/2d;
			
			double dxx = DOG[s][x+1][y]-2*DOG[s][x][y]+DOG[s][x-1][y];
			double dyy = DOG[s][x][y+1]-2*DOG[s][x][y]+DOG[s][x][y-1];
			double dss = DOG[s+1][x][y]-2*DOG[s][x][y]+DOG[s-1][x][y];
			
			double dxy = ((DOG[s][x+1][y+1]-DOG[s][x-1][y+1])-(DOG[s][x+1][y-1]-DOG[s][x-1][y-1]))/4d;
			double dxs = ((DOG[s+1][x+1][y]-DOG[s+1][x-1][y])-(DOG[s-1][x+1][y]-DOG[s-1][x-1][y]))/4d;
			double dys = ((DOG[s+1][x][y+1]-DOG[s+1][x][y-1])-(DOG[s-1][x][y+1]-DOG[s-1][x][y-1]))/4d;

					
			double[][] Hessian = {{dxx, dxy, dxs},{dxy, dyy, dys},{dxs, dys, dss}};
			double[] Jacobian = {dx, dy, ds};
			Matrix HessianInverse=null;
			try {
		    HessianInverse = new Matrix(Hessian).inverse();
			}catch(Exception e) {
				return new double[] {0,0,0,0};
			}
		    
		    double X0x = HessianInverse.get(0,0)*Jacobian[0]+HessianInverse.get(0,1)*Jacobian[1]+HessianInverse.get(0,2)*Jacobian[2];
		    double X0y = HessianInverse.get(1,0)*Jacobian[0]+HessianInverse.get(1,1)*Jacobian[1]+HessianInverse.get(1,2)*Jacobian[2];
		    double X0s = HessianInverse.get(2,0)*Jacobian[0]+HessianInverse.get(2,1)*Jacobian[1]+HessianInverse.get(2,2)*Jacobian[2];
		    
		    double contrast = DOG[s][x][y] + 0.5d*(Jacobian[0]*X0x+Jacobian[1]*X0y+Jacobian[2]*X0s);
		    
		    return new double[] {X0x,X0y,X0s,contrast};

		}
		
		public boolean ImprovedPosition(point p)  throws Exception  {
			double data[]=null;
			if((int)p.x<w&&(int)p.y<h&&(int)p.x>0&&(int)p.y>0)
			data = getDerivativeVector(p);
			else return false;
			double again[] = null;
			
			if(Math.abs(data[0])>0.5||Math.abs(data[1])>0.5) {//3 attempts to recompute
				p.x+=data[0]; p.y+=data[1];
				if((int)p.x<w&&(int)p.y<h&&(int)p.x>0&&(int)p.y>0)
					again = getDerivativeVector(p);
					else return false;
				
				if(Math.abs(again[0])>0.5||Math.abs(again[1])>0.5) {
					p.x+=again[0]; p.y+=again[1];
					if((int)p.x<w&&(int)p.y<h&&(int)p.x>0&&(int)p.y>0)
						again = getDerivativeVector(p);
						else return false;
					
					if(Math.abs(again[0])>0.5||Math.abs(again[1])>0.5) {
						p.x+=again[0]; p.y+=again[1]; 
						if((int)p.x<w&&(int)p.y<h&&(int)p.x>0&&(int)p.y>0)
						again = getDerivativeVector(p);
						else return false;
						
						if(Math.abs(again[0])>0.5||Math.abs(again[1])>0.5) {
							return false;
						}
						
					}
					
				}
				
				
			}
					    
		    if(data[3]<KEYPOINT_INTENSITY_THRESHOLD)return false;
		    else {
		    	p.x+=data[0]; p.y+=data[1];
		    	return true;}

			
		}
		
		public void subPixelPositions() throws Exception {
			
		    for(int y =0;y<keypoints.size();y++)
			    if(!ImprovedPosition(keypoints.get(y))) {
			    	keypoints.remove(y);y--;
			    }
			
			
			
			
//			for( int i = 0; i < keypoints.size(); i++ ) {
//	        	int x = Math.round(keypoints.get(i).x);
//	        	int y = Math.round(keypoints.get(i).y);
//	        	if(x==0||y==0||x==w||y==h) {keypoints.remove(i); i--;break;}
//	        	long o = original[x][y];
//	        	long r = original[x+1][y]; 
//	        	long a = original[x][y-1]; 
//				double intensity = Math.sqrt(Math.pow(o-r, 2)+Math.pow(o-a, 2));
//				
//				if(intensity<threshold) {keypoints.remove(i); i--;}
//	        	
//	        }
			
		}
		
		public void discardEdges() {
			
			for( int i = 0; i < keypoints.size(); i++ ) {
	        	int x = Math.round(keypoints.get(i).x);
	        	int y = Math.round(keypoints.get(i).y);
	        	long value = harisValues[x][y];
	        	if(value<THRESHOLD_EDGES){keypoints.remove(i); i--;}
	        	
	        }
		}
		
		public void DescribeKeypoints(int[][][] guassian) 
		{
			this.feturePositions= new ArrayList<point>();
			this.fetures = new ArrayList<double[]>();
			for(int k=0;k<keypoints.size();k++){
				Feature f = new Feature(keypoints.get(k),this,guassian[keypoints.get(k).scalespace-1]);
				this.fetures.add(f.describe(guassian[keypoints.get(k).scalespace-1],f.Baserotation));
				feturePositions.add(keypoints.get(k));
				if(f.second) {
					this.fetures.add(f.describe(guassian[keypoints.get(k).scalespace-1],f.SecondBaserotation));
					feturePositions.add(keypoints.get(k));
				}
				
			}
		}
		
		
	}
	
	
	
}
