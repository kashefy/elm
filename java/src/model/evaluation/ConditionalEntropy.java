/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.evaluation;

/**
 *
 * @author woodstock
 */
public class ConditionalEntropy {
    
    public static double calculate(double [] par_values){
        
        double normCondEntropy = 0;
        int nofValues = par_values.length;
        for(int i=0; i<nofValues; i++){
            
            double p = par_values[i];
            if(p != 0)
                normCondEntropy += p * Math.log(p);
        }
        return -normCondEntropy;
    }
    
    public ConditionalEntropy(){
        
    }
    
}
