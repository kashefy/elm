
package model.encoding;

/*
 * Simple Population Code
 * input: n binary variables xk
 * output: transformation f(xk) = (-1, u1, u2)
 * u1 = 1, if xk = 1
 * u2 = 1, if xk = 0
 */

public class SimplePopulationCode extends AbstractPopulationCode
{
    public static int NO_BIAS = -1;
    protected int m_biasValue;
    protected int m_biasIndex;

    /*
     * @param par_nofInput - number of x input variables
     * @param par_fanOut - number of output variables mapped to each input variable. Currently only works with par_nfanOut = 3;
     */
    public void init(int par_nofInputs, int par_nfanOut){

        super.init(par_nofInputs, par_nfanOut);

        // each input x is coded by three y-Neurons. Additionally
        // one bias term is given
        m_biasValue = 1;
        m_biasIndex = NO_BIAS;
        //m_biasIndex = -1;// set to -1 if you're not using a bias term
        
        m_nofNodes += (m_biasIndex == NO_BIAS)? 0:1;            
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

        if(m_biasIndex != NO_BIAS){
            stateVals[ m_biasIndex ] = m_biasValue; // bias term?
        }

        int j; // index of output nodes
        for(int i = 0; i < m_nofInputs; ++i){

                j = i*m_nfanOut;
                if(m_biasIndex != NO_BIAS)
                    ++j;
                
                // output nodes per input node are antognistic
                switch(m_nfanOut){
                    
                    case 2:
                        stateVals[j  ] =   par_inputVals[i];
                        stateVals[j+1] = 1-par_inputVals[i];
                        
                        break;
                    case 3:
                        stateVals[j  ] = par_inputVals[i];
                        stateVals[j+1] = -1;    // constant for this node
                        stateVals[j+2] = 1-par_inputVals[i];
                        
                        break;
                    default: // case: 1
                        stateVals[j] = par_inputVals[i];
                }

        }

        return stateVals;
    }
    
    /*
     * @see AbstractPopulationCode#getStateOfNeurons(int[])
     */
    @Override
    public int[] calcStateOfNeurons(int[] par_inputVals) 
    {
        // since we have binary x values, each variable xk is associated
        // with n yi values. n is determined by fanOut value
        int[] stateVals = new int[ m_nofNodes ];

        if(m_biasIndex != NO_BIAS){
            stateVals[ m_biasIndex ] = m_biasValue; // bias term?
        }

        int j; // index of output nodes
        for(int i = 0; i < m_nofInputs; ++i){

                j = i*m_nfanOut;
                if(m_biasIndex != NO_BIAS)
                    ++j;
                
                // output nodes per input node are antognistic
                switch(m_nfanOut){
                    
                    case 2:
                        stateVals[j  ] =   par_inputVals[i];
                        stateVals[j+1] = 1-par_inputVals[i];
                        
                        break;
                    case 3:
                        stateVals[j  ] = par_inputVals[i];
                        stateVals[j+1] = -1;    // constant for this node
                        stateVals[j+2] = 1-par_inputVals[i];
                        
                        break;
                    default: // case: 1
                        stateVals[j] = par_inputVals[i];
                }

        }

        return stateVals;
    }

    /*
     * @see AbstractPopulationCode#getNumberInputNeurons()
     */
    @Override
    int getnofNodes(){

        return m_nofNodes;
    }

    @Override
    int[] getMapping(){

        int [] arrMapping = new int[]{m_nofNodes, m_nfanOut, m_biasValue};
        return arrMapping;
    }

    /*
     * calculate optimal weights based on parameters of bayesian network.
     * Formulas can be deduced using marginalization
     * @see AbstractPopulationCode#getOptWeightValues(double[], double[], double)
     */
    @Override
    double[] getOptWeightValues(double[] alphas, double[] betas, double pReward) 
    {
            double[] weights = new double[7];

            double mult = pReward/(1-pReward);
            // p(x2=1|r=1):
            double v1 = betas[2] * (1-alphas[1]) + betas[3] * alphas[1];
            // p(x2=1|r=0):
            double v2 = (1-alphas[0]) * betas[0] + alphas[0] * betas[1];

            weights[1] = Math.log(alphas[1]/alphas[0] * mult);
            weights[3] = Math.log((1-alphas[1])/(1-alphas[0]) * mult);
            weights[4] = Math.log(v1/v2 * mult);
            weights[6] = Math.log((1-v1)/(1-v2) * mult);

            weights[0] = Math.log(mult);
            weights[2] = Math.log(mult);
            weights[5] = Math.log(mult);

            return weights;
    }

            /*
     * some test function
     */
    @Override
    int test(){

        return 0;
    }

    public int getBiasIndex(){

        return m_biasIndex;
    }

     /*
     * 
     */
    public SimplePopulationCode() {

    }
}
