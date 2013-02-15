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

public abstract class AbstractFeatureMap {
    
    //protected int [] m_arrDims;
    protected int m_nofFeatureSets; 
    
    abstract public void setParams(FeatureMapParams params);
    
    abstract public void init();
    
    abstract public void setStimulus(ComplexArray par_stimulus);
    
    abstract public ComplexArray [] getResponse();
    
    abstract public ComplexArray [] convolve();
    
    public ComplexArray [] convolve(ComplexArray par_stimulus){
        
        return null;
    }
    
    public ComplexArray convolve(RealArray par_stimulus, int par_filterIndex){
        
        return null;
    }
        
    public ComplexArray convolve(ComplexArray par_stimulus, int par_filterIndex){
        
        return null;
    } 
    
    abstract public int getNofFeatureSets();
    
    abstract public void calcCentreSurroundDiff();
    
    //abstract public int[] getInputDimensions();
    //abstract public int[] getFeatureDimensions();
    
}
