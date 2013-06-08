/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.features;

/**
 *
 * @author woodstock
 */
import model.utils.ModelUtils;
import org.shared.array.*;

import model.utils.files.*;

import java.util.Random;
import java.io.File;

import java.awt.Graphics2D;
import java.awt.geom.*;
        
public class OrientationMapTest {
    
    
    public void test1(){
        
       OrientationMap mapOrient = new OrientationMap();
       
       int supportRadius = 128;
       int nofScales = 1;
       double scalingFactor = 1.5;
       double orientationResolution = 60.0;
       double frequency = 1.0;
       double elongation = 0.65;
       
       mapOrient.setStartingScale(0.2*16/supportRadius);
       mapOrient.init(supportRadius,nofScales,scalingFactor,orientationResolution,frequency,elongation);
       mapOrient.saveFilters(".\\output");
       
       ComplexArray stimulus;
       int nofRows = supportRadius*2+1;
       int nofCols = supportRadius*2+1;
       int x0 = supportRadius;
       int y0 = supportRadius;
       int band = 2;
       double angleInDeg = 120.0;
       double [] stimulusRealVals = genBarStimulusVals(nofRows,nofCols,x0,y0,band,angleInDeg);
           
       double [] stimulusComplexVals = ModelUtils.real2Complex(stimulusRealVals);
       stimulus = new ComplexArray(stimulusComplexVals,nofRows,nofCols,2);
       
//       double [] a = new double[]{1,2,3,4,5,6,7,8,9,10,11,12,70,80,90,100,110,120,10,20,30,40,50,60};
//       int [] dimdim = new int[]{6,2,2};   
//       stimulus = new ComplexArray(a,dimdim);
//       int [] dd = stimulus.dims();
//       System.out.printf("b =%n%s%n", stimulus.toString());
//       System.out.println(stimulus.get(0,0,0));
//       System.out.println(stimulus.get(0,0,1));
//       System.out.println(stimulus.get(0,5,0));
//       System.out.println(stimulus.get(0,5,1));  
//       System.out.println(stimulus.get(1,5,0));
//       System.out.println(stimulus.get(1,5,1));   
//       System.out.println(stimulus.get(new int[]{1,5,1}));   
     
       ComplexArray[] output = mapOrient.convolve(stimulus);
       String strOutputFilepath = ".\\output\\";
       String strExt = ".csv";
       
       FileIO.saveArrayToCSV(stimulusRealVals,nofRows,nofCols,strOutputFilepath + "stimRe" + strExt);
       
       for(int i=0; i<output.length; i++){
           
           String strPrefix;
                   
           strPrefix = Integer.toString(i);
           strPrefix = strOutputFilepath.concat(strPrefix);
           FileIO.saveComplexArrayToCSV(output[i],strPrefix + "out" + strExt);
           int [] nDims = output[i].torRe().dims(); 
           FileIO.saveArrayToCSV(output[i].torRe().values(),nDims[FileIO.DIM_INDEX_ROWS],nDims[FileIO.DIM_INDEX_COLS],strPrefix + "outRe" + strExt);
           FileIO.saveArrayToCSV(output[i].torIm().values(),nDims[FileIO.DIM_INDEX_ROWS],nDims[FileIO.DIM_INDEX_COLS],strPrefix + "outIm" + strExt);
       }
    }
    
    // testing response to two filters with +/- theta angles
    public void test2(){
        
       OrientationMap mapOrient = new OrientationMap();
       
       int supportRadius = 64;
       int nofScales = 2;
       double scalingFactor = 1.5;
       double [] orientationsInDeg = new double[]{-90,90.0};
       double frequency = 1.0;
       double elongation = 0.65;
       
       mapOrient.init(supportRadius,nofScales,scalingFactor,orientationsInDeg,frequency,elongation);
       mapOrient.saveFilters(".\\output");
       
       ComplexArray stimulus;
       int nofRows = supportRadius*2+1;
       int nofCols = supportRadius*2+1;
       int x0 = supportRadius;
       int y0 = supportRadius;
       int band = 2;
       double angleInDeg = -60.0;
       double [] stimulusRealVals = genBarStimulusVals(nofRows,nofCols,x0,y0,band,angleInDeg); 
       double [] stimulusComplexVals = ModelUtils.real2Complex(stimulusRealVals);
       stimulus = new ComplexArray(stimulusComplexVals,nofRows,nofCols,2);
     
       ComplexArray[] output = mapOrient.convolve(stimulus);
       String strOutputFilepath = ".\\output\\";
       String strExt = ".csv";
       
       FileIO.saveArrayToCSV(stimulusRealVals,nofRows,nofCols,strOutputFilepath + "stimRe" + strExt);
       
       for(int i=0; i<output.length; i++){
           
           String strPrefix;
                   
           strPrefix = Integer.toString(i);
           strPrefix = strOutputFilepath.concat(strPrefix);
           FileIO.saveComplexArrayToCSV(output[i],strPrefix + "out" + strExt);
           int [] nDims = output[i].torRe().dims(); 
           FileIO.saveArrayToCSV(output[i].torRe().values(),nDims[FileIO.DIM_INDEX_ROWS],nDims[FileIO.DIM_INDEX_COLS],strPrefix + "outRe" + strExt);
           FileIO.saveArrayToCSV(output[i].torIm().values(),nDims[FileIO.DIM_INDEX_ROWS],nDims[FileIO.DIM_INDEX_COLS],strPrefix + "outIm" + strExt);
       }
    }
    
    // testing filter paramters (scale)
    public void test3(){

       OrientationMap mapOrient = new OrientationMap();
       
       int supportRadius = 16;
       int nofScales = 4;
       double scalingFactor = 1.5;
       double [] orientationsInDeg = new double[]{0};
       double frequency = 1.0;
       double elongation = 0.65;
       
       mapOrient.setStartingScale(0.2*supportRadius/16);
       mapOrient.init(supportRadius,nofScales,scalingFactor,orientationsInDeg,frequency,elongation);
       mapOrient.saveFilters(".\\output");
       
       ComplexArray stimulus;
       int nofRows = supportRadius*2+1;
       int nofCols = supportRadius*2+1;
       int x0 = supportRadius;
       int y0 = supportRadius;
       int band = 2;
       double angleInDeg = 120.0;
       double [] stimulusRealVals = genBarStimulusVals(nofRows,nofCols,x0,y0,band,angleInDeg);
           
       double [] stimulusComplexVals = ModelUtils.real2Complex(stimulusRealVals);
       stimulus = new ComplexArray(stimulusComplexVals,nofRows,nofCols,2);
       
//       double [] a = new double[]{1,2,3,4,5,6,7,8,9,10,11,12,70,80,90,100,110,120,10,20,30,40,50,60};
//       int [] dimdim = new int[]{6,2,2};   
//       stimulus = new ComplexArray(a,dimdim);
//       int [] dd = stimulus.dims();
//       System.out.printf("b =%n%s%n", stimulus.toString());
//       System.out.println(stimulus.get(0,0,0));
//       System.out.println(stimulus.get(0,0,1));
//       System.out.println(stimulus.get(0,5,0));
//       System.out.println(stimulus.get(0,5,1));  
//       System.out.println(stimulus.get(1,5,0));
//       System.out.println(stimulus.get(1,5,1));   
//       System.out.println(stimulus.get(new int[]{1,5,1}));   
       
//       RealArray r = new RealArray(stimulusRealVals,nofRows,nofCols);
//       ComplexArray rff = r.rfft();
//       ComplexArray sff = stimulus.fft();
//       System.out.printf("rff =%n%s%n", r.toString());
//       System.out.printf("rff =%n%s%n", rff.toString()); 
//       System.out.printf("c =%n%s%n", sff.toString()); 
     
       ComplexArray[] output = mapOrient.convolve(stimulus);
       String strOutputFilepath = ".\\output\\";
       String strExt = ".csv";
       
       FileIO.saveArrayToCSV(stimulusRealVals,nofRows,nofCols,strOutputFilepath + "stimRe" + strExt);
       
       for(int i=0; i<output.length; i++){
           
           String strPrefix;
                   
           strPrefix = Integer.toString(i);
           strPrefix = strOutputFilepath.concat(strPrefix);
           FileIO.saveComplexArrayToCSV(output[i],strPrefix + "out" + strExt);
           int [] nDims = output[i].torRe().dims(); 
           FileIO.saveArrayToCSV(output[i].torRe().values(),nDims[FileIO.DIM_INDEX_ROWS],nDims[FileIO.DIM_INDEX_COLS],strPrefix + "outRe" + strExt);
           FileIO.saveArrayToCSV(output[i].torIm().values(),nDims[FileIO.DIM_INDEX_ROWS],nDims[FileIO.DIM_INDEX_COLS],strPrefix + "outIm" + strExt);
       }
    }
    
    public static void testFilterResponseBarStim(){
        
        String strFeatParamFilename = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\input\\barSet\\featParamFileSimplePatterns.txt";
       String strFilterOutputDir = ".\\data\\output\\barSet\\FilterResponse\\";
       OrientationMap mapOrient = new OrientationMap();
              
       FeatureMapParams featParams = new FeatureMapParams();
       featParams.load(strFeatParamFilename);
       
       //mapOrient.setStartingScale(0.2*16/supportRadius);
       mapOrient.setParams(featParams);
       mapOrient.setStartingScale(0.2*featParams.getSupportRadius()/16);
       mapOrient.init();
       //mapOrient.init(supportRadius,nofScales,scalingFactor,orientationResolution,frequency,elongation);
       mapOrient.saveFilters(strFilterOutputDir);
       
       ComplexArray stimulus;
       int nofRows = featParams.getSupportRadius()*2+1;
       int nofCols = featParams.getSupportRadius()*2+1;
       int x0 = featParams.getSupportRadius();
       int y0 = featParams.getSupportRadius();
       int band = 3;
       double angleInDeg = 52.0;
       double [] stimulusRealVals = genBarStimulusVals(nofRows,nofCols,x0,y0,band,angleInDeg);
           
       double [] stimulusComplexVals = ModelUtils.real2Complex(stimulusRealVals);
       stimulus = new ComplexArray(stimulusComplexVals,nofRows,nofCols,2);
     
       mapOrient.setStimulus(stimulus);
       ComplexArray[] output = mapOrient.convolve();
       String strOutputFilepath = strFilterOutputDir;
       String strExt = ".csv";
       
       FileIO.saveArrayToCSV(stimulusRealVals,nofRows,nofCols,strOutputFilepath + "stimRe" + strExt);
       
       for(int i=0; i<output.length; i++){
           
           String strPrefix;
                   
           strPrefix = Integer.toString(i);
           strPrefix = strOutputFilepath.concat(strPrefix);
           FileIO.saveComplexArrayToCSV(output[i],strPrefix + "out" + strExt);
           int [] nDims = output[i].torRe().dims(); 
           FileIO.saveArrayToCSV(output[i].torRe().values(),nDims[FileIO.DIM_INDEX_ROWS],nDims[FileIO.DIM_INDEX_COLS],strPrefix + "outRe" + strExt);
           FileIO.saveArrayToCSV(output[i].torIm().values(),nDims[FileIO.DIM_INDEX_ROWS],nDims[FileIO.DIM_INDEX_COLS],strPrefix + "outIm" + strExt);
       }
    }
    
    public static void testFilterResponseBarStimWithNoise(){
        
        String strFeatParamFilename = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\input\\barNoiseSet\\featParamFile.txt";
       String strFilterOutputDir = ".\\data\\output\\barNoiseSet\\FilterResponse\\";
       OrientationMap mapOrient = new OrientationMap();
              
       FeatureMapParams featParams = new FeatureMapParams();
       featParams.load(strFeatParamFilename);
       
       //mapOrient.setStartingScale(0.2*16/supportRadius);
       //featParams.setOrientation(20);
       featParams.setSupportRadius(32);
       mapOrient.setParams(featParams);
       mapOrient.setStartingScale(0.2*featParams.getSupportRadius()/16);
       mapOrient.init();
       //mapOrient.init(supportRadius,nofScales,scalingFactor,orientationResolution,frequency,elongation);
       mapOrient.saveFilters(strFilterOutputDir);
       
       ComplexArray stimulus;
       int nofRows = featParams.getSupportRadius()*2+1;
       int nofCols = featParams.getSupportRadius()*2+1;
       int x0 = featParams.getSupportRadius();
       int y0 = featParams.getSupportRadius();
       int band = 3;
       double angleInDeg = 45.0;
       double [] stimulusRealVals = genBarStimulusVals(nofRows,nofCols,x0,y0,band,angleInDeg);
       
       // add noise by flipping values
       Random generator = new Random();
       for(int i=0; i<nofRows*nofCols; i++){
           double r = generator.nextGaussian();
           if(Math.abs(r)>2)
               stimulusRealVals[i] = 1-stimulusRealVals[i];
       }
           
       double [] stimulusComplexVals = ModelUtils.real2Complex(stimulusRealVals);
       stimulus = new ComplexArray(stimulusComplexVals,nofRows,nofCols,2);
     
       mapOrient.setStimulus(stimulus);
       ComplexArray[] output = mapOrient.convolve();
       String strOutputFilepath = strFilterOutputDir;
       String strExt = ".csv";
       
       FileIO.saveArrayToCSV(stimulusRealVals,nofRows,nofCols,strOutputFilepath + "stimRe" + strExt);
       
       for(int i=0; i<output.length; i++){
           
           String strPrefix;
                   
           strPrefix = Integer.toString(i);
           strPrefix = strOutputFilepath.concat(strPrefix);
           FileIO.saveComplexArrayToCSV(output[i],strPrefix + "out" + strExt);
           int [] nDims = output[i].torRe().dims(); 
           FileIO.saveArrayToCSV(output[i].torRe().values(),nDims[FileIO.DIM_INDEX_ROWS],nDims[FileIO.DIM_INDEX_COLS],strPrefix + "outRe" + strExt);
           FileIO.saveArrayToCSV(output[i].torIm().values(),nDims[FileIO.DIM_INDEX_ROWS],nDims[FileIO.DIM_INDEX_COLS],strPrefix + "outIm" + strExt);
       }
    }
    
    public static void testFilterResponseMNIST(){
        
        String strInputDataDir = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\input\\MNIST\\";
        File inputDataDir = new File(strInputDataDir);
        String strFeatParamFilename = "featParamFile.txt";
        File featParamFile = new File(inputDataDir, "featParamFile.txt");
      
        String strMainOnputDir = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\output\\MNIST\\";
        File mainOnputDir = new File(strMainOnputDir);
        String strFilterOutputDir = "filterResponse\\";
        File filterOutputDir = new File(mainOnputDir, strFilterOutputDir);
       
        OrientationMap mapOrient = new OrientationMap();

        FeatureMapParams featParams = new FeatureMapParams();
        featParams.load(featParamFile.getPath());

        //mapOrient.setStartingScale(0.2*16/supportRadius);
        mapOrient.setParams(featParams);
        //mapOrient.setStartingScale(0.15*featParams.getSupportRadius()/16);
        mapOrient.init();
        mapOrient.saveFilters(filterOutputDir.getPath());

        ComplexArray stimulus;
        AbstractDataLoader dataLoader = new DataLoaderImageSetCSV();
        dataLoader.setParams(inputDataDir.getPath());
        dataLoader.init();
        dataLoader.load();
        int [] stimDims = dataLoader.getDims();
        int nofRows = stimDims[FileIO.DIM_INDEX_ROWS];
        int nofCols = stimDims[FileIO.DIM_INDEX_COLS];
        double [] stimulusRealVals = dataLoader.getSample(0);
        //       for(int i=0; i<nofRows * nofCols; i++){
        //            
        //            stimulusRealVals[i] = 1-stimulusRealVals[i];
        //        }
        double [] stimulusComplexVals = ModelUtils.real2Complex(stimulusRealVals);
        stimulus = new ComplexArray(stimulusComplexVals,nofRows,nofCols,2);

        mapOrient.setStimulus(stimulus);
        ComplexArray[] output = mapOrient.convolve();
        RealArray[] outputRect = rectify(output);

        String strExt = ".csv";

        FileIO.saveArrayToCSV(stimulusRealVals,nofRows,nofCols,new File(filterOutputDir, "stimRe" + strExt).getPath());

        for(int i=0; i<output.length; i++){

           String strPrefix;

           strPrefix = Integer.toString(i);
           FileIO.saveComplexArrayToCSV(output[i],new File(filterOutputDir, strPrefix + "out" + strExt).getPath());
           int [] nDims = output[i].torRe().dims(); 
           //FileIO.saveArrayToCSV(output[i].torRe().values(),nDims[FileIO.DIM_INDEX_ROWS],nDims[FileIO.DIM_INDEX_COLS],new File(filterOutputDir, strPrefix + "outRe" + strExt).getPath());
           //FileIO.saveArrayToCSV(output[i].torIm().values(),nDims[FileIO.DIM_INDEX_ROWS],nDims[FileIO.DIM_INDEX_COLS],new File(filterOutputDir, strPrefix + "outIm" + strExt).getPath());
           FileIO.saveArrayToCSV(outputRect[i].values(),nDims[FileIO.DIM_INDEX_ROWS],nDims[FileIO.DIM_INDEX_COLS],new File(filterOutputDir, strPrefix + "outAbs" + strExt).getPath());
        }
    }
    
    public static double [] genBarStimulusVals(int par_nofRows, int par_nofCols, int par_x0, int par_y0, int par_band, double par_angleInDeg){
        
        // draw Line2D.Double
//        int x1 = 0;
//        int y1 = 0;
//        int x2 = par_nofRows-1;
//        int y2 = par_nofCols-1;
//        Graphics2D g2 = new Graphics2D();
//        g2.draw(new Line2D.Double(x1, y1, x2, y2));

        int nofElements = par_nofRows*par_nofCols;
        double [] barStimulus = new double[ nofElements ];
        double angle = Math.toRadians(par_angleInDeg);
        int evenBand;
        if (par_band == 1 || par_band % 2 != 0)
            evenBand = 0;
        else
            evenBand = 1;
        
        
        for(int i=0; i<nofElements; i++){
            
            barStimulus[ i ] = 0;
        }
        
        if (par_angleInDeg == 0 || par_angleInDeg == 360){ // horizontals
            
            for(int by=-par_band/2+par_y0; by<=par_band/2+par_y0-evenBand; by++){
                   
                int rowIndex = by*par_nofCols;
                if (by>=0 && by<par_nofRows){
                    for(int x=0; x<par_nofCols; x++){

                        barStimulus[ rowIndex+x ] = 1;
                    }
                }
            }
        }
        // verticals
        else if (par_angleInDeg == 90 || par_angleInDeg == -90 || par_angleInDeg == 270){
            
            for(int y=0; y<par_nofRows; y++){
             
                int rowOffset = y*par_nofCols;
                for(int bx=-par_band/2+par_x0; bx<=par_band/2+par_x0-evenBand; bx++){
                
                    if (bx>=0 && bx<par_nofCols){
                        barStimulus[ rowOffset+bx ] = 1;
                    }
                }
            }
        }
        else if((Math.abs(par_angleInDeg) > 45 && par_angleInDeg < 135) ||
                (par_angleInDeg > 225 && par_angleInDeg < 315)){
            
            // current implementation leaves gaps for near vertical bars, need to deal with case separately.
            barStimulus = genBarStimulusVals(par_nofCols, par_nofRows, par_y0, par_x0, par_band, par_angleInDeg-90);
            barStimulus = rotate90AntiClockwise(barStimulus);
//            
//            // current implementation leaves gaps for near vertical bars
//            par_angleInDeg = Math.abs(par_angleInDeg) - 90;
//            
//            for(int x=0; x<par_nofCols; x++){
//
//                int deltaX = x-par_x0;
//                int y = (int)Math.floor(par_y0 - deltaX * Math.tan(angle));
//
//                for(int bx=-par_band/2+x; bx<=par_band/2+x-evenBand; bx++){
//                    for(int by=-par_band/2+y; by<=par_band/2+y-evenBand; by++){
//
//                        // avoid index out of bounds
//                        if(bx>=0 && bx<par_nofCols && by>=0 && by<par_nofRows){
//                            barStimulus[ bx*par_nofRows+by ] = 1;
//                        }   
//                    }
//                }  
//            }
        }
        else{    // reamaining angles
            for(int x=0; x<par_nofCols; x++){

                int deltaX = x-par_x0;
                int y = (int)Math.floor(par_y0 - deltaX * Math.tan(angle));

                for(int bx=-par_band/2+x; bx<=par_band/2+x-evenBand; bx++){
                    for(int by=-par_band/2+y; by<=par_band/2+y-evenBand; by++){

                        // avoid index out of bounds
                        if(bx>=0 && bx<par_nofCols && by>=0 && by<par_nofRows){
                            barStimulus[ by*par_nofCols+bx ] = 1;
                        }   
                    }
                }  
            }
        }
        return barStimulus;
    }
        
    
    public static double [] rotate90AntiClockwise(double [] par_values){
        
        // assuming square images
        int nofRows = (int)Math.floor(Math.sqrt(par_values.length));
        int nofCols = nofRows;
        double [] rotatedVals = new double [nofRows * nofCols];
        
        for(int r=0; r<nofRows; r++){
            
            int rowOffset = r*nofCols;
            for(int c=0; c<nofCols; c++){
                
                double val = par_values[ rowOffset+c ];
                rotatedVals[(nofRows-c-1)*nofRows + r] = val;
            }
        }
        //FileIO.saveArrayToCSV(par_values, nofRows, nofCols, "a.txt");
        //FileIO.saveArrayToCSV(rotatedVals, nofRows, nofCols, "b.txt");
        return rotatedVals;
    }
    
    public static RealArray[] rectify(ComplexArray[] par_arrResponseRef){
        
        int nofResponses = par_arrResponseRef.length;
        double minVal = 0;
        double maxVal = 0; 
        RealArray[] responsesRectfied = new RealArray[ nofResponses ];
        Random generator = new Random();
        
        for(int i=0; i<nofResponses; i++){
            
            //System.out.printf("b =%n%s%n", par_arrResponseRef[i].toString());
            responsesRectfied[i] = par_arrResponseRef[i].torAbs();
            double candidate = responsesRectfied[i].aMax();
            if(candidate > maxVal){
                
                maxVal = candidate;
            }
            else if(candidate == maxVal){
                
                maxVal = (generator.nextBoolean())? maxVal : candidate;
            }
        }
        
        if(maxVal > 0)
            maxVal = 1.0/maxVal;
        
        for(int i=0; i<nofResponses; i++){
            
            responsesRectfied[i] = responsesRectfied[i].uMul(maxVal);
        }      
        
        return responsesRectfied;
    }
    
    public static void testCalcNeighOrientDist(){
        
        int nofRows = 5;
        int nofCols = 5;
        RealArray input = new RealArray(nofRows,nofCols);
        int nofOrientations = 6;
        int orientationResolution = 30;
        
        Random generator = new Random();
        
        for(int r=0; r<nofRows; r++){
            
            for(int c=0; c<nofCols; c++){
                
                int orientation = generator.nextInt(nofOrientations)*orientationResolution;
                input.set(orientation, r,c);
            }
        }
        System.out.printf("i =%n%s%n", input.toString());
        RealArray orientDists = OrientationMap.calcNeighOrientDist(input,2);
        System.out.printf("d =%n%s%n", orientDists.toString());
        RealArray orientDistsVar = ModelUtils.calcNeighVar(orientDists, 2);
        System.out.printf("dv =%n%s%n", orientDistsVar.toString());
    }
    
    public OrientationMapTest(){
        
        
    }
}
