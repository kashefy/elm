/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.encoding;

/**
 *
 * @author woodstock
 */
import model.*;
import model.utils.*;

public class MAXPopulationCode extends SimplePopulationCode{
    
    /*
     * @see AbstractPopulationCode#getStateOfNeurons(double[])
     */
    @Override
    double[] calcStateOfNeurons(double[] par_inputVals) 
    {
        // since we have binary x values, each variable xk is associated
        // with n yi values. n is determined by fanOut value
        double[] stateVals = new double[ m_nofNodes ];
        
        if(m_nofNodes-par_inputVals.length == 1){
            
            stateVals[ m_biasIndex ] = m_biasValue; // bias term?
        }

        double [] popCode = new double[ m_nofNodes ];
        
        int maxIndex = encode(par_inputVals, popCode);
        if(m_biasIndex == -1){
            
            // keep maxIndex as is
        }
        else if(m_biasIndex == 0){
            
            maxIndex++; 
        }
        else if(m_biasIndex == m_nofNodes-1){
            
            // keep maxIndex as is
        }
        else if(maxIndex == m_biasIndex){
            
            maxIndex++;
        }
        
        stateVals[ maxIndex ] = 1;

        return stateVals;
    }
    
    // select strongest to activate and shut down the rest
    // output nodes per input node are antognistic
    // return index of strongest popCode for input set
    private int encode(double [] par_inputs, double [] par_elementPopCode){
        
        double [] popCode = par_elementPopCode;
        
        int nofInputs = par_inputs.length;
        int nofNodes = nofInputs;
        //popCode = new double[ nofNodes ];

        // j: index of input AND output nodes
        // select strongest to activate and shut down the rest
        // output nodes per input node are antognistic

        int j=0;
        int boundIndex = nofNodes;
        int maxIndex = 0;
        double maxVal = par_inputs[j++]; //-1;
        for(; j<boundIndex; j++){

            //double candidate = Math.abs(par_inputs[j]);
            double candidate = par_inputs[j];
            if(candidate > maxVal){

                maxVal = candidate;
                maxIndex = j;
            }
            
            // don't need to set zero values, can assume they are already set to zero.
            // when created, arrays are automatically initialized with the default value of their type 
            // popCode[ j ] = 0.0;
        }
        popCode[ maxIndex ] = 1.0;
        return maxIndex;
    }
    
    public MAXPopulationCode(){
        
        m_biasIndex = 0; 
        m_biasValue = 1;             
    }
}
