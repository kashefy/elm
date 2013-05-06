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
    public double[] genPopCode(double [] par_inputVals){
        
        double [] popCodeStateValsDistr;
        popCodeStateValsDistr = popCoder.calcStateOfNeuronsDistr(par_inputVals);

        //be extra cautious and create an explicit copy
        m_popCodeStateVals = new double[ popCodeStateValsDistr.length ];
        System.arraycopy(popCodeStateValsDistr, 0, m_popCodeStateVals, 0, popCodeStateValsDistr.length);
        return popCodeStateValsDistr;
    }
    
    /**
     * assumes population code has already been generated via genPopCode()
     * @param input is unused
     * @return spike trains from pre-generated population code
     */
    @Override
    public int[][] genSpikeTrains(double [] par_inputVals){
        
        int nofNodes = popCoder.getnofNodes();
        int [][] spikeTrains;
        //int [][] spikeTrains = new int[ nofNodes ][ m_durationInMilSec ];
        spikeTrains = popCoder.sampleStateOfNeurons(m_popCodeStateVals, (int)(m_durationInMilSec/m_deltaT_milSec));
                             
        return spikeTrains;
    }
    
    public EncoderSoftMax(){
        
    }
}
