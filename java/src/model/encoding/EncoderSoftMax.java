/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.encoding;

/**
 *
 * @author woodstock
 */
public class EncoderSoftMax extends Encoder{
    
    /**
     * generates population code and corresponding spike trains
     * @param stimulus to encode
     * @return spike trains
     */
    @Override
    public int[][] encode(double [] par_inputVals){
        
        genPopCode(par_inputVals); // will be done inside genSpikeTrains()
        int [][] spikeTrains = genSpikeTrains(par_inputVals);
        return spikeTrains;
    }
    
    /**
     * generates population code only
     * @param stimulus
     * @return population code
     */
    @Override
    public double[] genPopCode(double [] par_input_vals){
        
        double [] pop_code_distr_vals = popCoder.calcStateOfNeuronsDistr(par_input_vals);

        //be extra cautious and create an explicit copy
        m_popCodeStateVals = new double[ pop_code_distr_vals.length ];
        System.arraycopy(pop_code_distr_vals, 0, m_popCodeStateVals, 0, pop_code_distr_vals.length);
        return pop_code_distr_vals;
    }
    
    /**
     * assumes the  code has already been generated via genPopCode()
     * @param input is unused
     * @return spike trains from pre-generated population code
     */
    @Override
    public int[][] genSpikeTrains(double [] par_inputVals){
        
        int [][] spikeTrains;
        spikeTrains = popCoder.sampleStateOfNeurons(m_popCodeStateVals, nof_spikes_per_train);
        return spikeTrains;
    }
    
    public EncoderSoftMax(){
        
    }
}
