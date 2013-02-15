/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.attention;

/**
 *
 * @author woodstock
 */
public abstract class AbstractSaliencyMeasure {
    
    public void setParams(SaliencyParams par_params){
        
    }
    
    public abstract void init();
    
    public abstract void eval(double [] par_inputs);
    
    public abstract double measure(int [] locRef);
    
    public abstract double [] measure(int [][] par_arr_locRef);
    
    public abstract double[] getSaliency();
    
    public abstract void save(String par_strPath);
}
