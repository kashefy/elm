/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.features;

/**
 *
 * @author woodstock
 */

// sst imports
import org.shared.array.*;
import org.shared.image.filter.Gabor;

import java.io.*;
import model.utils.ModelUtils;
import model.utils.files.FileIO;

public class OrientationMap extends AbstractFeatureMap{
    
    private Gabor [] m_gaborFilterBank;
    private ComplexArray [] m_gaborFilterBankFFT;
    private ComplexArray [] m_filterBankToSize;
    private ComplexArray [] m_filterBankToSizeFFT;
    private int m_nofScales;
    private double m_startingScale;
    private double m_scalingFactor;
    
    private int m_orientationInitMode;
    private int m_nofOrientations;
    private double m_orientationResolution;
    private double [] m_orientationsInDeg;
    
    private int m_supportRadius;
    private double m_frequency;
    private double m_elongation;
    
    ComplexArray m_currentStimulus;
    ComplexArray m_currentStimulusFFT;
    
    public void setParams(FeatureMapParams params){
        
        m_supportRadius = params.getSupportRadius();
        m_nofScales = params.getNofScales();
        m_scalingFactor = params.getScalingFactor();
        
        m_nofOrientations = -1;
        m_orientationInitMode = params.getOrientationInitMode();        
        if(m_orientationInitMode == FeatureMapParams.ORIENT_INIT_MODE_ARBITRARY){
            
            m_orientationResolution = -1;
            m_nofOrientations = params.getOrientationsInDeg().length;
            m_orientationsInDeg = new double[ m_nofOrientations ];
            System.arraycopy(params.getOrientationsInDeg(), 0, m_orientationsInDeg, 0, m_nofOrientations);
                        
        }
        else if(m_orientationInitMode == FeatureMapParams.ORIENT_INIT_MODE_RANGE){
            
            m_orientationResolution = params.getOrientationResolution();
        }
        
        m_frequency = params.getGaborFrequency();
        m_elongation = params.getElongation();
    }
    
    private void initOrientations(){
        
        if(m_orientationInitMode == FeatureMapParams.ORIENT_INIT_MODE_ARBITRARY){
            
            // already taken care of in setParams()
        }
        else if(m_orientationInitMode == FeatureMapParams.ORIENT_INIT_MODE_RANGE){
            
            m_nofOrientations = (int)Math.floor(180/m_orientationResolution);
            m_orientationsInDeg = new double [ m_nofOrientations ];
            double thetaInDeg = 0;
            for(int o=0; o<m_nofOrientations; ++o){
                
                m_orientationsInDeg[o] = thetaInDeg;
                thetaInDeg += m_orientationResolution;
            }
        }        
    }
    
    public void init(){
        
        initOrientations();
        m_nofFeatureSets = m_nofScales * m_nofOrientations;
        m_gaborFilterBank = new Gabor[ m_nofFeatureSets ];
        m_gaborFilterBankFFT = new ComplexArray[ m_nofFeatureSets ]; 
        m_filterBankToSize = new ComplexArray [ m_nofFeatureSets ];
        m_filterBankToSizeFFT = new ComplexArray [ m_nofFeatureSets ];

        double scale = m_startingScale;
        for(int s=0; s<m_nofScales; s++){
            
            double thetaInRad = 0;
            int rowIndex = s*m_nofOrientations;
            for(int o=0; o<m_nofOrientations; ++o){
                
                thetaInRad = Math.toRadians(m_orientationsInDeg[o]);
                m_gaborFilterBank[ rowIndex + o ] = new Gabor(m_supportRadius,thetaInRad,scale,m_frequency,m_elongation);
                m_gaborFilterBankFFT[ rowIndex + o ] = m_gaborFilterBank[ rowIndex + o ].fft();
            }
            
            scale *= m_scalingFactor;
        }
        
    }
    
    public void init(int par_supportRadius, 
            int par_nofScales, double par_scalingFactor, 
            double par_orientationResolution, 
            double par_frequency, 
            double par_elongation){
        
        m_supportRadius = par_supportRadius;
        m_nofScales = par_nofScales;
        m_scalingFactor = par_scalingFactor;
        m_orientationResolution = par_orientationResolution;
        m_frequency = par_frequency;
        m_elongation = par_elongation;
        
        m_nofOrientations = (int)Math.floor(180/m_orientationResolution);
        m_orientationsInDeg = new double [ m_nofOrientations ];
        m_nofFeatureSets = m_nofScales * m_nofOrientations;
        m_gaborFilterBank = new Gabor[ m_nofFeatureSets ];
        
        double scale = m_startingScale;
        for(int s=0; s<m_nofScales; ++s){
            
            double thetaInDeg = 0;
            double thetaInRad;
            int rowIndex = s*m_nofOrientations;
            for(int o=0; o<m_nofOrientations; ++o){
                
                m_orientationsInDeg[o] = thetaInDeg;
                thetaInRad = Math.toRadians(thetaInDeg);
                m_gaborFilterBank[ rowIndex + o ] = new Gabor(m_supportRadius,thetaInRad,scale,m_frequency,m_elongation);
                
                thetaInDeg += m_orientationResolution;
            }
            
            scale *= m_scalingFactor;
        }
    }
    
    
    public void init(int par_supportRadius, 
            int par_nofScales, double par_scalingFactor, 
            double [] par_orientations, 
            double par_frequency, 
            double par_elongation){
        
        m_supportRadius = par_supportRadius;
        m_nofScales = par_nofScales;
        m_scalingFactor = par_scalingFactor;
        
        
        m_orientationResolution = -1;
        m_orientationsInDeg = new double[ par_orientations.length ];
        System.arraycopy(par_orientations, 0, m_orientationsInDeg, 0, par_orientations.length);
        m_nofOrientations = m_orientationsInDeg.length;
        
        m_nofFeatureSets = m_nofScales * m_nofOrientations;
        m_frequency = par_frequency;
        m_elongation = par_elongation;
        
        m_gaborFilterBank = new Gabor[ m_nofFeatureSets ];
        
        double scale = m_startingScale;
        for(int s=0; s<m_nofScales; ++s){
            
            double thetaInRad = 0;
            int rowIndex = s*m_nofOrientations;
            for(int o=0; o<m_nofOrientations; ++o){
                
                thetaInRad = Math.toRadians(m_orientationsInDeg[o]);
                m_gaborFilterBank[ rowIndex + o ] = new Gabor(m_supportRadius,thetaInRad,scale,m_frequency,m_elongation);
            }
            
            scale *= m_scalingFactor;
        }
    }
    
    public void saveFilters(String par_strOutputDir){
        
        File outputDir = new File(par_strOutputDir);
        int nofFilters = m_gaborFilterBank.length;
        String strPrefix;
        
        // create a file with filter attributes
        try (PrintWriter pw = new PrintWriter(new FileWriter(new File(outputDir, "filterAttr.txt")))){
            
            for(int i=0; i<nofFilters; ++i){

                pw.println(m_gaborFilterBank[i].toString());

            }
        }
         catch (Exception e){
                  
                System.err.println("Error: " + e.getMessage());
         }
        
        String strExt = ".csv"; // file extension

        for(int i=0; i<nofFilters; ++i){
            
            strPrefix = Integer.toString(i);
            ComplexArray filter = new ComplexArray(m_gaborFilterBank[i]);        
                
            FileIO.saveComplexArrayToCSV(filter, new File(outputDir, strPrefix + "filter" + strExt).getPath());
            int [] nDims = filter.torRe().dims();
            FileIO.saveArrayToCSV(filter.torRe().values(),nDims[0],nDims[1], new File(outputDir, strPrefix + "filterRe" + strExt).getPath());
            FileIO.saveArrayToCSV(filter.torIm().values(),nDims[0],nDims[1], new File(outputDir, strPrefix + "filterIm" + strExt).getPath());
        }
    }
    
    @Override
    public void setStimulus(ComplexArray par_stimulus){
        
        m_currentStimulus = par_stimulus;
        m_currentStimulusFFT = par_stimulus.fft();
    }
    
    @Override
    public ComplexArray [] getResponse(){
        
        return null;
    }
    
    @Override
    // will pad filters if necessary
    public ComplexArray [] convolve(){

            ComplexArray [] arr_output = null;
            if(m_currentStimulus == null || m_currentStimulusFFT == null)
                return null;

            int nof_rows_stim = m_currentStimulus.dims()[ FileIO.DIM_INDEX_ROWS ];
            int nof_cols_stim = m_currentStimulus.dims()[ FileIO.DIM_INDEX_COLS ];
            arr_output = new ComplexArray[ m_nofFeatureSets ];

            for(int i=0; i<m_nofFeatureSets; ++i){

                // check if stimulus and filter have the same dimensions
                ComplexArray filterToUse = m_gaborFilterBank[i];
                ComplexArray filterToUseFFT = m_gaborFilterBankFFT[i];
                int [] filterDims =  filterToUse.dims();
                int dRows = filterDims[ FileIO.DIM_INDEX_ROWS ] - nof_rows_stim;
                int dCols = filterDims[ FileIO.DIM_INDEX_COLS ] - nof_cols_stim;

                if(dRows != 0 || dCols != 0){

                    ComplexArray alternateFilter = m_filterBankToSize[i];
                    ComplexArray alternateFilterFFT = m_filterBankToSizeFFT[i];
                    if(alternateFilter == null){

                        // instantiate
                        m_filterBankToSize[i] = ModelUtils.pad2D(filterToUse, m_currentStimulus.dims());
                        alternateFilter = m_filterBankToSize[i];
                        
                        alternateFilterFFT = alternateFilter.fft();
                        m_filterBankToSizeFFT[i] = alternateFilterFFT;
                    }
                    else{

                        // verify
                        filterDims =  alternateFilter.dims();
                        dRows = filterDims[0] - nof_rows_stim;
                        dCols = filterDims[1] - nof_cols_stim;

                        if(dRows != 0 || dCols != 0){

                            m_filterBankToSize[i] = ModelUtils.pad2D(filterToUse, m_currentStimulus.dims());
                            alternateFilter = m_filterBankToSize[i];
                            
                            alternateFilterFFT = alternateFilter.fft();
                            m_filterBankToSizeFFT[i] = alternateFilterFFT;
                        }

                    }
                    filterToUse = alternateFilter;
                    filterToUseFFT = alternateFilterFFT;
                }

                arr_output[ i ] = m_currentStimulusFFT.eMul(filterToUse.fft()).ifft().fftShift();
                //arrOutput[ i ] = m_currentStimulusFFT.eMul(filterToUseFFT).ifft().fftShift();
                //arrOutput[ i ] = m_currentStimulusFFT.eMul(filterToUse.fft()).ifft();
            }
            return arr_output;
    }
        
    @Override
    // will pad filters if necessary
    public ComplexArray [] convolve(ComplexArray par_stimulus){
        
        int nofFilters = m_gaborFilterBank.length;
        int nofRowsStim = par_stimulus.dims()[ FileIO.DIM_INDEX_ROWS ];
        int nofColsStim = par_stimulus.dims()[ FileIO.DIM_INDEX_COLS ];
        ComplexArray [] arrOutput = new ComplexArray[ nofFilters ];
        
        m_filterBankToSize = new ComplexArray [ nofFilters ];
        
        for(int i=0; i<nofFilters; ++i){
            
            // check if stimulus and filter have the same dimensions
            ComplexArray filterToUse = m_gaborFilterBank[i];
            int [] filterDims =  filterToUse.dims();
            int dRows = filterDims[ FileIO.DIM_INDEX_ROWS ] - nofRowsStim;
            int dCols = filterDims[ FileIO.DIM_INDEX_COLS ] - nofColsStim;
            
            if(dRows != 0 || dCols != 0){
                
                ComplexArray alternateFilter = m_filterBankToSize[i];
                if(alternateFilter == null){
                    
                    // instantiate
                    m_filterBankToSize[i] = ModelUtils.pad2D(filterToUse, par_stimulus.dims());
                    alternateFilter = m_filterBankToSize[i];
                }
                else{
                    
                    // verify
                    filterDims =  alternateFilter.dims();
                    dRows = filterDims[0] - nofRowsStim;
                    dCols = filterDims[1] - nofColsStim;
                    
                    if(dRows != 0 || dCols != 0){
                        
                        m_filterBankToSize[i] = ModelUtils.pad2D(filterToUse, par_stimulus.dims());
                        alternateFilter = m_filterBankToSize[i];
                    }
                }
                filterToUse = alternateFilter;
            }
        
            arrOutput[ i ] = par_stimulus.fft().eMul(filterToUse.fft()).ifft();
        }
        return arrOutput;
    }
    
    @Override
    public ComplexArray convolve(RealArray par_stimulus, int par_filterIndex){
        
        // although RealArray come with rfft() only half of the columns are present,
        // we need to convert it to a complex array first to preserve dimensions of its fourier transform
        ComplexArray complexStimulus = ModelUtils.realArrayToComplex(par_stimulus);
        return convolve(complexStimulus, par_filterIndex);
    }
        
    @Override
    public ComplexArray convolve(ComplexArray par_stimulus, int par_filterIndex){
        
        ComplexArray output;
        output = par_stimulus.fft().eMul(m_gaborFilterBank[ par_filterIndex ].fft()).ifft();
        return output;
    }
    
    public int setStartingScale(double par_startingScale){
        
        if(par_startingScale <= 0){
            return 1;
        }
        else{
            m_startingScale = par_startingScale;
        }
        return 0;
    }
    
    public int getNofFeatureSets(){
        
        return m_nofFeatureSets;
    }
    
    @Override
    public void calcCentreSurroundDiff(){
        
    }
    
    public static RealArray calcNeighOrientDist(RealArray par_input, int par_neighborhood){
        
        int [] dims = par_input.dims();
        
        int nofRows = dims[ FileIO.DIM_INDEX_ROWS ];
        int nofCols = dims[ FileIO.DIM_INDEX_COLS ];
        
        RealArray neighDist = new RealArray(dims);
        
        for(int r=0; r<nofRows; ++r){
            
            int rStartiIncl = r-par_neighborhood;
            if(rStartiIncl < 0){
                rStartiIncl = 0;
            }
            
            int rEndiExcl = r+par_neighborhood+1;
            if(rEndiExcl > nofRows){
                rEndiExcl = nofRows;
            }
            
            for(int c=0; c<nofCols; ++c){
                
                int cStartiIncl = c-par_neighborhood;
                if(cStartiIncl < 0){
                    cStartiIncl = 0;
                }
            
                int cEndiExcl = c+par_neighborhood+1;
                if(cEndiExcl > nofCols){
                    cEndiExcl = nofCols;
                }
                RealArray sub = par_input.subarray(rStartiIncl,rEndiExcl,cStartiIncl,cEndiExcl);
                double meanOrient = sub.aMean();
                double currOrient = par_input.get(r, c);
                double newOrient = Math.abs(currOrient - meanOrient);
                if(newOrient > 90){
                    newOrient -= 90;
                }
                neighDist.set(newOrient, r, c);
            }
        }
        
        return neighDist;
        
    }

    // constructor
    public OrientationMap(){
        
        m_startingScale = 0.2;
        m_nofFeatureSets = 0;
    }
}
