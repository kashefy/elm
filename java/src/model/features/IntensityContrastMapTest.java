/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.features;

/**
 *
 * @author woodstock
 */
import org.shared.array.*;
import org.shared.image.filter.GaborCircular;

import java.io.*;
import model.utils.ModelUtils;
import model.utils.files.FileIO;
import model.utils.files.DataLoaderImageSetCSV;
import org.shared.image.filter.DerivativeOfGaussian;
import org.shared.image.filter.LaplacianOfGaussian;

public class IntensityContrastMapTest {
    
    // problem: blob at corners
    /*
     * t = 1-abs(y-mean(mean(y))) > thr
    private static RealArray rectify(RealArray par_responseRef){
        
        double threshold = 0.9;
        double mean = par_responseRef.aMean();
        par_responseRef = par_responseRef.uAdd(-mean);
        
        RealArray ones = new RealArray(par_responseRef.dims());
        ones.uFill(1.0);
        par_responseRef = par_responseRef.uAbs();
        par_responseRef = ones.eSub(par_responseRef);
        
        double [] values = par_responseRef.values();
        
        int nofRows = par_responseRef.dims()[ FileIO.DIM_INDEX_ROWS ];
        int nofCols = par_responseRef.dims()[ FileIO.DIM_INDEX_COLS ];
        int nofElements = nofRows*nofCols; 
        for(int i=0; i<nofElements; i++){
            
            double val = values[i];
            if(val<threshold){
                
                values[i] = 0;
            }
            else{
                
                //values[i] = (val-threshold)/(1-threshold);
            }
        }
        
        
        return par_responseRef;
    }
     * 
     */
    
    // 1-abs(y) > thr. doesn't work for 1-stim (black surround)
    private static RealArray rectify(RealArray par_responseRef){
        
        double threshold = 0.8;
        //double mean = par_responseRef.aMean();
        //par_responseRef = par_responseRef.uAdd(-mean);
        
        RealArray ones = new RealArray(par_responseRef.dims());
        ones.uFill(1.0);
        par_responseRef = par_responseRef.uAbs();
        par_responseRef = ones.eSub(par_responseRef);
        
        double [] values = par_responseRef.values();
        
        int nofRows = par_responseRef.dims()[ FileIO.DIM_INDEX_ROWS ];
        int nofCols = par_responseRef.dims()[ FileIO.DIM_INDEX_COLS ];
        int nofElements = nofRows*nofCols; 
        for(int i=0; i<nofElements; i++){
            
            double val = values[i];
            if(val<threshold){
                
                values[i] = 0;
            }
        }
        return par_responseRef;
    }
    
        public static void test1(){
        
        String strInputPath = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\input\\MNIST\\";
        String strOutputPath = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\output\\MNIST\\filterResponse\\";
        
        
        int supportRadius = 14;
        double scale = 0.3;
        double frequency = 1.0;
        
        GaborCircular gaborCirc = new GaborCircular(supportRadius, scale, frequency);
        
        int nofRowsStim = supportRadius*2+1;
        int nofColsStim = supportRadius*2+1;
        double [][] arrStimuliVals = new double [4][ nofRowsStim * nofColsStim ];
        int band = 2;
        double [] stimulusVals = new double [ nofRowsStim * nofColsStim ];
        for(int r=0; r<nofRowsStim; r++){
            
            for(int c=0; c<nofColsStim; c++){
                                
                if(c<=11)
                    stimulusVals[ r*nofColsStim+c ] = 1.0;
            }
        }
        arrStimuliVals[0] = stimulusVals;
        stimulusVals = new double [ nofRowsStim * nofColsStim ];
        for(int r=0; r<nofRowsStim; r++){
            
            for(int c=0; c<nofColsStim; c++){
                
                if(c >= nofColsStim/2-band && c <= nofColsStim/2+band && 
                        r >= nofRowsStim/2-band && r <= nofRowsStim/2+band){
                
                    stimulusVals[ r*nofColsStim+c ] = 1.0;
                        }
                
                if(c >= 23-band && c <= 23+band && 
                        r >= 23-band && r <= 23+band){
                
                    stimulusVals[ r*nofColsStim+c ] = 1.0;
                        }
                
                if(c<=14)
                    stimulusVals[ r*nofColsStim+c ] = 1.0;
            }
        }
        arrStimuliVals[1] = stimulusVals;
        
        DataLoaderImageSetCSV dataLoader = new DataLoaderImageSetCSV();
        dataLoader.setParams(strInputPath);
        dataLoader.init();
        dataLoader.load();
        double[] temp = dataLoader.getSample(0);
        int origNofRows = 28;
        int origNofCols = 28;
        arrStimuliVals[2] = new double [ nofRowsStim * nofColsStim ];
        // quick crop
        for(int r=0; r<origNofRows; r++){
            
            for(int c=0; c<origNofCols; c++){
                
                arrStimuliVals[2][r*nofColsStim+c] = temp[r*origNofCols+c];
            }
        }
        arrStimuliVals[3] = new double [ nofRowsStim * nofColsStim ];
        for(int i=0; i<nofRowsStim * nofColsStim; i++){
            
            arrStimuliVals[3][i] = 1-arrStimuliVals[2][i];
        }
       
        
        for(int i=0; i<arrStimuliVals.length; i++){
        
            double [] stimulusValsComplex;
            stimulusValsComplex = ModelUtils.real2Complex(arrStimuliVals[i]);
            int [] dims = new int[]{nofRowsStim, nofColsStim, 2};
            ComplexArray stimulus = new ComplexArray(stimulusValsComplex, dims);

            ComplexArray response = stimulus.fft().eMul(gaborCirc.fft()).ifft().fftShift();
            
            String strPrefix = Integer.toString(i);

            FileIO.saveArrayToCSV(stimulus.torRe().values(), nofRowsStim, nofColsStim, strOutputPath + strPrefix + "stimRe.csv");
            
            FileIO.saveArrayToCSV(gaborCirc.torRe().values(), nofRowsStim, nofColsStim, strOutputPath + strPrefix + "filterRe.csv");
            FileIO.saveArrayToCSV(gaborCirc.torIm().values(), nofRowsStim, nofColsStim, strOutputPath + strPrefix + "filterIm.csv");
            
            FileIO.saveArrayToCSV(response.torRe().values(), nofRowsStim, nofColsStim, strOutputPath + strPrefix + "outRe.csv");
            FileIO.saveArrayToCSV(response.torIm().values(), nofRowsStim, nofColsStim, strOutputPath + strPrefix + "outIm.csv");
        }
        
    }
        
    public static void test2(){
        
        String strInputPath = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\input\\MNIST\\";
        String strOutputPath = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\output\\MNIST\\filterResponse\\";
        
        int supportRadius = 14;
        double scale = 1.0;
        double frequency = 1.0;
        
        //GaborCircular filter = new GaborCircular(supportRadius, scale, frequency);
        LaplacianOfGaussian filterLap  = new LaplacianOfGaussian(supportRadius, scale);
        int [] dimsFilter = new int[]{filterLap.dims()[0], filterLap.dims()[1], 2};
        double [] filterLapComplexVals = ModelUtils.real2Complex(filterLap.values());
        ComplexArray filter = new ComplexArray(filterLapComplexVals, dimsFilter);
        //filter.

        LaplacianOfGaussian filterLap2  = new LaplacianOfGaussian(supportRadius*10, scale);
        LaplacianOfGaussian filterLap3  = new LaplacianOfGaussian(supportRadius*10, scale/10);
        LaplacianOfGaussian filterLap4  = new LaplacianOfGaussian(supportRadius*10, scale/100);
        LaplacianOfGaussian filterLap5  = new LaplacianOfGaussian(supportRadius*10, scale/1000);
        
        FileIO.saveArrayToCSV(filterLap2.values(), 281, 281, strOutputPath + "0filterRe.csv");
        FileIO.saveArrayToCSV(filterLap3.values(), 281, 281, strOutputPath + "1filterRe.csv");
        FileIO.saveArrayToCSV(filterLap4.values(), 281, 281, strOutputPath + "2filterRe.csv");
        FileIO.saveArrayToCSV(filterLap5.values(), 281, 281, strOutputPath + "3filterRe.csv");
        
        RealArray diff = filterLap2.eSub(filterLap3);
        
        double sumOfDiff = diff.aSum();
        
        diff = filterLap4.eSub(filterLap5);
        
        double sumOfDiff2 = diff.aSum();
        
        int nofRowsStim = supportRadius*2+1;
        int nofColsStim = supportRadius*2+1;
        double [][] arrStimuliVals = new double [4][ nofRowsStim * nofColsStim ];
        int band = 2;
        double [] stimulusVals = new double [ nofRowsStim * nofColsStim ];
        for(int r=0; r<nofRowsStim; r++){
            
            for(int c=0; c<nofColsStim; c++){
                                
                if(c<=11)
                    stimulusVals[ r*nofColsStim+c ] = 1.0;
            }
        }
        arrStimuliVals[0] = stimulusVals;
        stimulusVals = new double [ nofRowsStim * nofColsStim ];
        for(int r=0; r<nofRowsStim; r++){
            
            for(int c=0; c<nofColsStim; c++){
                
                if(c >= nofColsStim/2-band && c <= nofColsStim/2+band && 
                        r >= nofRowsStim/2-band && r <= nofRowsStim/2+band){
                
                    stimulusVals[ r*nofColsStim+c ] = 1.0;
                        }
                
                if(c >= 23-band && c <= 23+band && 
                        r >= 23-band && r <= 23+band){
                
                    stimulusVals[ r*nofColsStim+c ] = 1.0;
                        }
                
                if(c<=14)
                    stimulusVals[ r*nofColsStim+c ] = 1.0;
            }
        }
        arrStimuliVals[1] = stimulusVals;
        
        DataLoaderImageSetCSV dataLoader = new DataLoaderImageSetCSV();
        dataLoader.setParams(strInputPath);
        dataLoader.init();
        dataLoader.load();
        double[] temp = dataLoader.getSample(0);
        int origNofRows = 28;
        int origNofCols = 28;
        arrStimuliVals[2] = new double [ nofRowsStim * nofColsStim ];
        // quick crop
        for(int r=0; r<origNofRows; r++){
            
            for(int c=0; c<origNofCols; c++){
                
                arrStimuliVals[2][r*nofColsStim+c] = temp[r*origNofCols+c];
            }
        }
        arrStimuliVals[3] = new double [ nofRowsStim * nofColsStim ];
        for(int i=0; i<nofRowsStim * nofColsStim; i++){
            
            arrStimuliVals[3][i] = 1-arrStimuliVals[2][i];
        }
       
        
        for(int i=0; i<arrStimuliVals.length; i++){
        
            double [] stimulusValsComplex;
            stimulusValsComplex = ModelUtils.real2Complex(arrStimuliVals[i]);
            int [] dims = new int[]{nofRowsStim, nofColsStim, 2};
            ComplexArray stimulus = new ComplexArray(stimulusValsComplex, dims);

            ComplexArray response = stimulus.fft().eMul(filter.fft()).ifft().fftShift();
            
            String strPrefix = Integer.toString(i);

            FileIO.saveArrayToCSV(stimulus.torRe().values(), nofRowsStim, nofColsStim, strOutputPath + strPrefix + "stimRe.csv");
            
            FileIO.saveArrayToCSV(filter.torRe().values(), nofRowsStim, nofColsStim, strOutputPath + strPrefix + "filterRe.csv");
            FileIO.saveArrayToCSV(filter.torIm().values(), nofRowsStim, nofColsStim, strOutputPath + strPrefix + "filterIm.csv");
            
            
            FileIO.saveArrayToCSV(response.torRe().values(), nofRowsStim, nofColsStim, strOutputPath + strPrefix + "outRe.csv");
            FileIO.saveArrayToCSV(response.torIm().values(), nofRowsStim, nofColsStim, strOutputPath + strPrefix + "outIm.csv");
        } 
    }
    
    public static void test3(){
        
        String strInputPath = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\input\\MNIST\\";
        String strOutputPath = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\output\\MNIST\\filterResponse\\";
        
        int supportRadius = 14;
        double scale = 1.0;
        
        DiffOfGaussians2DSq diffOfG = new DiffOfGaussians2DSq();
        diffOfG.setParams(supportRadius, scale);
        diffOfG.init();
        RealArray filterReal = diffOfG.getKernelRef();
        ComplexArray filter;
        filter = new ComplexArray(ModelUtils.real2Complex(filterReal.values()),supportRadius*2+1,supportRadius*2+1,2);
        
        
        int nofRowsStim = supportRadius*2+1;
        int nofColsStim = supportRadius*2+1;
        double [][] arrStimuliVals = new double [4][ nofRowsStim * nofColsStim ];
        int band = 2;
        double [] stimulusVals = new double [ nofRowsStim * nofColsStim ];
        for(int r=0; r<nofRowsStim; r++){
            
            for(int c=0; c<nofColsStim; c++){
                                
                if(c<=11)
                    stimulusVals[ r*nofColsStim+c ] = 1.0;
            }
        }
        arrStimuliVals[0] = stimulusVals;
        stimulusVals = new double [ nofRowsStim * nofColsStim ];
        for(int r=0; r<nofRowsStim; r++){
            
            for(int c=0; c<nofColsStim; c++){
                
                if(c >= nofColsStim/2-band && c <= nofColsStim/2+band && 
                        r >= nofRowsStim/2-band && r <= nofRowsStim/2+band){
                
                    stimulusVals[ r*nofColsStim+c ] = 1.0;
                        }
                
                if(c >= 23-band && c <= 23+band && 
                        r >= 23-band && r <= 23+band){
                
                    stimulusVals[ r*nofColsStim+c ] = 1.0;
                        }
                
                if(c<=14)
                    stimulusVals[ r*nofColsStim+c ] = 1.0;
            }
        }
        arrStimuliVals[1] = stimulusVals;
        
        DataLoaderImageSetCSV dataLoader = new DataLoaderImageSetCSV();
        dataLoader.setParams(strInputPath);
        dataLoader.init();
        dataLoader.load();
        double[] temp = dataLoader.getSample(2);
        int origNofRows = 28;
        int origNofCols = 28;
        arrStimuliVals[2] = new double [ nofRowsStim * nofColsStim ];
        // quick crop
        for(int r=0; r<origNofRows; r++){
            
            for(int c=0; c<origNofCols; c++){
                
                arrStimuliVals[2][r*nofColsStim+c] = temp[r*origNofCols+c];
            }
        }
        arrStimuliVals[3] = new double [ nofRowsStim * nofColsStim ];
        for(int i=0; i<nofRowsStim * nofColsStim; i++){
            
            arrStimuliVals[3][i] = 1-arrStimuliVals[2][i];
        }
       
        for(int i=0; i<arrStimuliVals.length; i++){
        
            double [] stimulusValsComplex;
            stimulusValsComplex = ModelUtils.real2Complex(arrStimuliVals[i]);
            int [] dims = new int[]{nofRowsStim, nofColsStim, 2};
            ComplexArray stimulus = new ComplexArray(stimulusValsComplex, dims);

            ComplexArray response = stimulus.fft().eMul(filter.fft()).ifft().fftShift();
            
            String strPrefix = Integer.toString(i);

            FileIO.saveArrayToCSV(stimulus.torRe().values(), nofRowsStim, nofColsStim, strOutputPath + strPrefix + "stimRe.csv");
            
            FileIO.saveArrayToCSV(filter.torRe().values(), nofRowsStim, nofColsStim, strOutputPath + strPrefix + "filterRe.csv");
            FileIO.saveArrayToCSV(filter.torIm().values(), nofRowsStim, nofColsStim, strOutputPath + strPrefix + "filterIm.csv");
                        
            RealArray responseReal = response.torRe();
            responseReal = rectify(responseReal);
                    
            FileIO.saveArrayToCSV(responseReal.values(), nofRowsStim, nofColsStim, strOutputPath + strPrefix + "outRe.csv");
            FileIO.saveArrayToCSV(response.torIm().values(), nofRowsStim, nofColsStim, strOutputPath + strPrefix + "outIm.csv");
        } 
    }
}
