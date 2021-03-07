import javax.imageio.ImageIO;
import javax.swing.JFrame;

import Jama.Matrix;

import java.awt.Canvas;
import java.awt.Graphics;
import java.awt.Image;
import java.awt.geom.AffineTransform;
import java.awt.image.AffineTransformOp;
import java.awt.image.BufferStrategy;
import java.awt.image.BufferedImage;
import java.awt.image.ColorConvertOp;
import java.awt.image.ColorModel;
import java.awt.image.WritableRaster;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.awt.Color;

public class Xd
{
	static int height;
	static int width;
	static String title;
	static JFrame frame;
	static Canvas canvas;
	static BufferStrategy bufferStrategy;
	static Graphics graphics;
	static boolean running;
	static int[][][] guassianLibrary;
	
	static int chosen4=13;
	static int matched=0;
	static ArrayList<Integer[]> matches=null;
	
	
	public static void main(String[] args)  throws Exception 
	{

		initiliseUI();
	    
	    //reading in the image
	    BufferedImage img=null, imgR = null, gray=null, grayR =null;
	    {
	    try {img = ImageIO.read(new File("C:\\Users\\Lenovo\\Desktop\\jpg.PNG"));} 
	    catch (IOException e) {System.out.print(e);}
	    gray = new BufferedImage(img.getWidth(),img.getHeight(),BufferedImage.TYPE_BYTE_GRAY);
	    ColorConvertOp op = new ColorConvertOp(img.getColorModel().getColorSpace(),gray.getColorModel().getColorSpace(),null);
	    op.filter(img,gray);
	    }
	    
	    {
		    try {imgR = ImageIO.read(new File("C:\\Users\\Lenovo\\Desktop\\jpgR.PNG"));} 
		    catch (IOException e) {System.out.print(e);}
		    grayR = new BufferedImage(imgR.getWidth(),imgR.getHeight(),BufferedImage.TYPE_BYTE_GRAY);
		    ColorConvertOp op = new ColorConvertOp(imgR.getColorModel().getColorSpace(),grayR.getColorModel().getColorSpace(),null);
		    op.filter(imgR,grayR);
		    }
	    int h = img.getHeight();
	    int w = img.getWidth();
	    
	    guassianLibrary = initialiseGuassianLibrary(7, 5, 1.6d, Math.sqrt(2)); 
	    int[][][] CircularGuassianLibrary = initialiseCircularGuassian(16, 2, 1.6d*1.5, Math.sqrt(2));
	    
	    int initDownSize = 1;
	    

	    
	    
//	    Octave octave1 = new Octave(img, 5, "1",(int)Math.pow(2, 0)*initDownSize);
//	    Octave octave2 = new Octave(img, 5, "2",(int)Math.pow(2, 0)*2);
//	    Octave octave3 = new Octave(img, 5, "3",(int)Math.pow(2, 2)*initDownSize);
	    Octave octave4 = new Octave(gray, 5, "4",(int)Math.pow(2, 2)*initDownSize);
	    Octave octave5 = new Octave(grayR, 5, "5",(int)Math.pow(2,2)*initDownSize);

//	    octave1.generate_guassians(guassianLibrary);
//	    octave2.generate_guassians(guassianLibrary);
//	    octave3.generate_guassians(guassianLibrary);
	    octave4.generate_guassians(guassianLibrary);
	    octave5.generate_guassians(guassianLibrary);
//
	    octave4.generateDOGs();
	    octave4.findKeypoints();
	    octave4.computeHarrisdeterminant();
	    octave4.discardEdges();
	    octave4.subPixelPositions();
	    octave4.DescribeKeypoints(CircularGuassianLibrary);
	    
//	    
	    octave5.generateDOGs();
	    octave5.findKeypoints();
	    octave5.computeHarrisdeterminant();
	    octave5.discardEdges();
	    octave5.subPixelPositions();
	    octave5.DescribeKeypoints(CircularGuassianLibrary);
	    
	    matches = new ArrayList<Integer[]>();
	    
	    for(int o=0;o<octave4.fetures.size();o++){
	    ArrayList<Double> distances = new ArrayList<Double>();
	    double[] o4vector = octave4.fetures.get(o);
	    double smallest=1000000000;
	    int smallestIndex=-1;
	    for(int i=0;i<octave5.fetures.size();i++) {
	    	double[] currentO5f = octave5.fetures.get(i);
	    	double distance=0;
	    	for(int r=0;r<128;r++) {
	    		distance+=Math.abs(o4vector[r]-currentO5f[r]);
	    	}
	    	distances.add(distance);
	    	
	    	if(distance<smallest) {
	    		smallest=distance;
	    		smallestIndex=i;
	    	}
	    		
	    }
	    if(smallest<50000)
	    matches.add(new Integer[] {(int)octave4.feturePositions.get(o).x,(int)octave4.feturePositions.get(o).y,
	    						   (int)octave5.feturePositions.get(smallestIndex).x,(int)octave5.feturePositions.get(smallestIndex).y});
	    }
	    
	    //octave4.fetures.get(initDownSize)
	    
	
	    
	    //BufferedImage haris = octave1.gradientX(sobelY);
	 
    
	    while (running) {
	        bufferStrategy = canvas.getBufferStrategy();
	        graphics = bufferStrategy.getDrawGraphics();
	        graphics.clearRect(0, 0, width, height);
	        graphics.setColor(Color.GREEN);
	  
	        
	        double highest = 0;
	        double lowest = 0;
	        for( int x = 0; x < octave4.w-10; x++ )
		        for( int y = 0; y < octave4.h-10; y++ ) {
		        	
		        	double value = octave4.guassianised[1][x][y];
		        	if (value>highest)highest=value;
		        	if (value<lowest)lowest=value;
		        	//graphics.setColor(new Color((int)normalize(value),(int)normalize(value),(int)normalize(value)));
//		        	graphics.setColor(Color.GREEN);
//		        	if(value<-100000)
//		        	graphics.drawLine(x, y ,x,y);
		        
		        	
		        }
	        
	        
	        
	        double fullRange = Math.abs(highest-lowest);
	        for( int x = 0; x < octave4.w-10; x++ )
		        for( int y = 0; y < octave4.h-10; y++ ) {
		        	
		        	long value = (long)((octave4.guassianised[1][x][y]-lowest)/(double)fullRange*255d);
		    
		        	graphics.setColor(new Color((int)(value),(int)(value),(int)(value)));
//		        	graphics.setColor(Color.GREEN);
//		        	if(value<-100000)
		        	graphics.drawLine(x, y ,x,y);
		        
		        	
		        }
	        
	        for(int i=0;i<octave4.feturePositions.size();i++) {
	        	if(octave4.feturePositions.get(i).scalespace==1)
		        	graphics.setColor(Color.CYAN);
		        	if(octave4.feturePositions.get(i).scalespace==2)
			        	graphics.setColor(Color.RED);

	        	int x = Math.round(octave4.feturePositions.get(i).x);
	        	int y = Math.round(octave4.feturePositions.get(i).y);
	        	graphics.drawLine(x, y ,x,y);
	        	if(i==chosen4) {
	        		graphics.drawLine(x-5, y ,x+5,y);
	        		graphics.drawLine(x, y-5 ,x,y+5);
	        	}
	        }
	        
	        
	        
	        
	        
	        highest = 0;
	        lowest = 0;
	        for( int x = 0; x < octave5.w-10; x++ )
		        for( int y = 0; y < octave5.h-10; y++ ) {
		        	
		        	double value = octave5.guassianised[1][x][y];
		        	if (value>highest)highest=value;
		        	if (value<lowest)lowest=value;
		        	//graphics.setColor(new Color((int)normalize(value),(int)normalize(value),(int)normalize(value)));
//		        	graphics.setColor(Color.GREEN);
//		        	if(value<-100000)
//		        	graphics.drawLine(x, y ,x,y);
		        
		        	
		        }
	        
//	        System.out.println("h: "+highest);
//	        System.out.println("l: "+lowest);
	        
	        
	        
	        fullRange = Math.abs(highest-lowest);
	        for( int x = 0; x < octave5.w-10; x++ )
		        for( int y = 0; y < octave5.h-10; y++ ) {
		        	
		        	long value = (long)((octave5.guassianised[1][x][y]-lowest)/(double)fullRange*255d);
		    
		        	graphics.setColor(new Color((int)(value),(int)(value),(int)(value)));
//		        	graphics.setColor(Color.GREEN);
//		        	if(value<-100000)
		        	graphics.drawLine(x+700, y ,x+700,y);
		        
		        	
		        }
	        
//	        
	        for(int i=0;i<octave5.feturePositions.size();i++) {
	        	graphics.setColor(Color.RED);

	        	if(octave5.feturePositions.get(i).scalespace==1)
	        	graphics.setColor(Color.CYAN);
	        	if(octave5.feturePositions.get(i).scalespace==2)
		        	graphics.setColor(Color.RED);

	        	int x = Math.round(octave5.feturePositions.get(i).x);
	        	int y = Math.round(octave5.feturePositions.get(i).y);
	        	graphics.drawLine(x+700, y ,x+700,y);
	        	
	        	if(i==matched) {
	        		graphics.drawLine(x-5+700, y ,x+5+700,y);
	        		graphics.drawLine(x+700, y-5 ,x+700,y+5);
	        	}
	        }
	        
	        //drawing matches
	        for(int m =0;m<matches.size();m++) {
	        	Integer[] curr = matches.get(m);
	        	graphics.drawLine(curr[0],curr[1],curr[2]+700,curr[3]);
	        }
	        
	        
//	      
	
//	        
	        bufferStrategy.show();
	        graphics.dispose();
	    }
	    
	}
	
	static void initiliseUI() {
		
	    title = "Test Window";
	    width = 1920;
	    height =width*9/16;
	    frame = new JFrame(title);
	    frame.setSize(width, height);
	    frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	    frame.setLocationRelativeTo(null);
	    frame.setResizable(false);
	    frame.setVisible(true);
	    canvas = new Canvas();
	    canvas.setSize(width, height);
	    canvas.setBackground(Color.BLACK);
	    canvas.setVisible(true);
	    canvas.setFocusable(false);
	    frame.add(canvas);
	    canvas.createBufferStrategy(3);

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
		
//		for(int i=0;i<5;i++) {
//			String ret ="";
//			for(int j=0;j<5;j++)
//			{ret+=(Double.toString(tiles[i][j].value())+", ");}
//			System.out.println(ret);
//		}
		
		
		double multiplier = 1/tiles[0][0].value();
		int[][] values = new int[n][n];
		for(int y=0;y<n;y++)
			for(int x=0;x<n;x++){
				values[x][y] = (int)Math.round(tiles[x][y].value()*multiplier);
			}
		//the tile I am checking now is supposed to return 1;
		
		
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

	
	static class Octave{
		
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
		
		static class point{
			float x,y;
			int scalespace;
			public point(int x,int y, int scalespace) {
				this.x=x;this.y=y;
				this.scalespace = scalespace;
			}
			
		}

		static class Feature{
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
				System.out.println("finished! - "+(r+1)+"/"+NUMBER_OF_GUASSIANS+", Octave:"+this.octave);
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

				    
				    double kappa = 0.16d;  // 0.04 - 0.15
				    double threshold_ = 0.5;  // 0.5

				 
				    long[][] determinant = subtract(multiply(gradient_xx, gradient_yy), multiply(gradient_xy, gradient_yx));
				    
				    long[][] trace = add(gradient_xx, gradient_yy);
				    
				    long[][] rCoefficient = subtract(determinant,multiplyScalar(multiply(trace, trace),(0.15d)));
				    
				    this.harisValues  = rCoefficient;
				    //origibal = trace

				    	    // // this function is a surrogate to compare the magnitudes of the eigenvalues
				    //subtractHarris(determinant, multiplyScalar(multiply(trace, trace),kappa));
		
				    
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
					    
		    if(data[3]<0.69302d)return false;
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
	        	if(value<-100000){keypoints.remove(i); i--;}
	        	
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