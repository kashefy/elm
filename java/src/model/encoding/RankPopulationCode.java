/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.encoding;

/**
 * by ranking on a [0,1] scale
 * @author woodstock
 */
import model.utils.ModelUtils;

public class RankPopulationCode extends GradientMAXPopulationCode{
    
    @Override
    public void init(int par_nofInputs, int par_nfanOut){

        super.init(par_nofInputs, par_nfanOut);
        assert m_biasIndex == -1;
    }
    
    /*
     * @see AbstractPopulationCode#getStateOfNeurons(double[])
     */
    @Override
    public double[] calcStateOfNeurons(double[] par_input_vals) 
    {
        assert par_input_vals.length*m_nfanOut == m_nofNodes;
        double[] state_vals = new double[ m_nofNodes ];
        
        int max_index = encode(par_input_vals, state_vals);
        
        return state_vals;
    }
    
    // pop code shows linear order of activation
    // return index of strongest node in input set
    private int encode(double [] par_inputs, double [] par_elementPopCode){
        
        int [] ranks = ModelUtils.argsort(par_inputs);
        for(int i=0, j=0; i<m_nofNodes; i+=m_nfanOut, ++j){
            
            par_elementPopCode[i+ranks[j]] = 1.0; // assuming par_elementPopCode is already all-zero
        }
        int argmax = ModelUtils.argmax(ranks);
        return argmax;
    }
    
    public RankPopulationCode(){
        
        super();
    }
}
