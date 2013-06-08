/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.features;

/**
 *
 * @author woodstock
 */
import org.shared.array.RealArray;
import org.shared.array.ComplexArray;
import model.utils.*;
import model.utils.files.FileIO;

public class IntensityContrastMap extends AbstractFeatureMap{
       
    private int m_supportRadius;
    private double m_scale;
    
    private DiffOfGaussians2DSq m_filter;
    private RealArray m_filterKernelReal;
    private ComplexArray m_filterKernelComplex;
    private ComplexArray m_filterKernelComplexFFT;
    
    private ComplexArray m_currentStimulus;
    private ComplexArray m_currentStimulusFFT;
    private ComplexArray [] m_arrResponse;
    
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
    public static RealArray rectify(RealArray par_responseRef){
        
        double threshold = 0.9;
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
        for(int i=0; i<nofElements; ++i){
            
            double val = values[i];
            if(val<threshold){
                
                values[i] = 0;
            }
        }
        return par_responseRef;
    }
    
    @Override
    public void setParams(FeatureMapParams params){
        
        m_supportRadius = 14;
        m_scale = 1.0;
        m_nofFeatureSets = 1;
        m_filter.setParams(m_supportRadius, m_scale);
    }
    
    @Override
    public void init(){

        m_filter.init();
        m_filterKernelReal = m_filter.getKernelRef();
        m_filterKernelReal = m_filterKernelReal.subarray(0,28,0,28);
        int nofRows = m_supportRadius*2;//+1;
        int nofCols = nofRows;
        m_filterKernelComplex = new ComplexArray(ModelUtils.real2Complex(m_filterKernelReal.values()),nofRows,nofCols,2);
        m_filterKernelComplexFFT = m_filterKernelComplex.fft();
    }
    
    @Override
    public void setStimulus(ComplexArray par_stimulus){
        
        m_currentStimulus = par_stimulus;
        m_currentStimulusFFT = par_stimulus.fft();
    }
    
    @Override
    public ComplexArray [] convolve(){
        
        m_arrResponse = new ComplexArray[ m_nofFeatureSets ];
        m_arrResponse[ m_nofFeatureSets-1 ] = m_currentStimulusFFT.eMul(m_filterKernelComplexFFT).ifft().fftShift();
        return m_arrResponse;
    }
    
    @Override
    public ComplexArray [] getResponse(){
        
        return m_arrResponse;
    }
    
    @Override
    public int getNofFeatureSets(){
        
        return m_nofFeatureSets;
    }
    
    @Override
    public void calcCentreSurroundDiff(){
        
    }
    
    public IntensityContrastMap(){
        
        m_filter = new DiffOfGaussians2DSq(); 
    }
    
}
