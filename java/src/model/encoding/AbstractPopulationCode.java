
package model.encoding;

/*
 * abstract class for different kinds of population codes
 * the input values x are coded in different ways to be represented by 
 * input neurons y
 */
public abstract class AbstractPopulationCode 
{
	protected int m_nofInputs;
        protected int m_nfanOut;
        protected int m_nofNodes;
        
        /*
	 * @param par_nofInput - number of x input variables
         * @param par_fanOut - number of output variables mapped to each input variable
	 */
        public void init(int par_nofInput, int par_nfanOut){
            
            m_nofInputs = par_nofInput;
            m_nfanOut   = par_nfanOut;
            m_nofNodes  = m_nofInputs * m_nfanOut;
        }
        
        public double[] calcStateOfNeuronsDistr(double[] inputs){
            
            double [] prelimVals = new double [ inputs.length ];
            System.arraycopy(inputs, 0, prelimVals, 0, inputs.length);
            return prelimVals;
        }
	
	/*
	 * returns an array containing the 'state' of the y-Neurons given the inputs x.
	 * When the state of one neuron is 1, the y-Neuron will then fire with its specified
	 * firing rate.
	 * 
	 * @param inputs - values from which to calculate the population code (x inputs)
	 */
	abstract double[] calcStateOfNeurons(double[] inputs);
        
        /*
	 * @see AbstractPopulationCode#getStateOfNeurons(double[])
	 */
        abstract int[] calcStateOfNeurons(int[] inputs);
        
        /*
         * 
         */
        public int[][] sampleStateOfNeurons(double[] par_index, int nofSamples){
            
            return null;
        }
	
	/*
	 * returns number of y-Neurons to express population code
	 */
	abstract int getnofNodes();
        
	/*
	 * returns mapping of population code, fanout and bias terms
	 */        
        abstract int[] getMapping();
	
	/*
	 * some test function
	 */
	abstract int test();
	
	/**
	 * returns optimal neural network weights dependent on the given parameters (parameters from
	 * bayesian network). 
	 * 
	 * @param alphas - alpha values of bayesian network
	 * @param betas - betas of bayesian network
	 * @param pReward - reward probability of bayesian network
	 */
	abstract double[] getOptWeightValues(double[] alphas, double[] betas, double pReward);
        

        
        public AbstractPopulationCode(){
		
	}

}
