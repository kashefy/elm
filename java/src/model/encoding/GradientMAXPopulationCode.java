/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.encoding;

/**
 * by ranking on a [0,1] scale
 * @author woodstock
 */

public class GradientMAXPopulationCode extends MAXPopulationCode{
    
    @Override
    public void init(int par_nofInputs, int par_nfanOut){

        super.init(par_nofInputs, par_nfanOut);
    }
    
    /*
     * @see AbstractPopulationCode#getStateOfNeurons(double[])
     */
    @Override
    public double[] calcStateOfNeurons(double[] par_inputVals) 
    {
        // since we have binary x values, each variable xk is associated
        // with n yi values. n is determined by fanOut value
        double[] stateVals = new double[ m_nofNodes ];
        int nofInputs = par_inputVals.length;
        double [] popCodeOrder = new double[ nofInputs ];
        
        int maxIndex = encode(par_inputVals, popCodeOrder);
        
        if(m_biasIndex == -1){
        
            stateVals = popCodeOrder;
        }
        else if(m_biasIndex == 0){
            
            // bias term placed at location 0
            System.arraycopy(popCodeOrder, 0, stateVals, 1, nofInputs);
        }
        else if(m_biasIndex == m_nofNodes-1){
            
            // bias term placed at location of last array element
            System.arraycopy(popCodeOrder, 0, stateVals, 0, nofInputs);
        }        
        else{
            
            // bias term placed somewhere in between
            // i    1 2 3 4
            // s    0 0 0 0 0
            // sbi  0 0 b 0 0
            //   i  0 1 2 3 4
            //   o  1 2 b 3 4
            System.arraycopy(popCodeOrder, 0, stateVals, 0, m_biasIndex);
            System.arraycopy(popCodeOrder, m_biasIndex, stateVals, m_biasIndex+1,nofInputs-m_biasIndex);
        }
        
//        for(int i=0; i<m_nofNodes; i++){
//        
//            System.out.print(m_nofNodes[i]+" ");
//        } 
        
        if(m_nofNodes-par_inputVals.length == 1){
            
            stateVals[ m_biasIndex ] = m_biasValue; // bias term?
        }
//        if(m_biasIndex == -1){
//            
//            // keep maxIndex as is
//        }
//        else if(m_biasIndex == 0){
//            
//            maxIndex++; 
//        }
//        else if(m_biasIndex == m_nofNodes-1){
//            
//            // keep maxIndex as is
//        }
//        else if(maxIndex == m_biasIndex){
//            
//            maxIndex++;
//        }
//        
//        stateVals[ maxIndex ] = 1;

        return stateVals;
    }
    
    // pop code shows linear order of activation
    // return index of strongest popCode for input set
    private int encode(double [] par_inputs, double [] par_elementPopCode){
        
        int nofInputs = par_inputs.length;
        int nofNodes = nofInputs;
        
        assert par_elementPopCode.length >= nofInputs;

        // find min and max
        int j=0; // index of input AND output nodes
        int boundIndex = nofNodes;
        int maxIndex = 0;
        int minIndex = 0;
        double maxVal = par_inputs[j++]; //-1;
        double minVal = maxVal;
        for(; j<boundIndex; ++j){

            //double candidate = Math.abs(par_inputs[j]);
            double candidate = par_inputs[j];
            if(candidate > maxVal){

                maxVal = candidate;
                maxIndex = j;
            }
            else if(candidate < minVal){

                minVal = candidate;
                minIndex = j;
            }
        }
        // determine linear order of elements
        double height = maxVal-minVal;
        
        for(int i=0; i<nofNodes; ++i){
            
            double val = par_inputs[i];
            val = (val-minVal)/height;
            par_elementPopCode[i] = val;
        }
        
        return maxIndex;
    }
    
    public GradientMAXPopulationCode(){
        
        super();
    }
}
