/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.encoding;

/**
 *
 * @author woodstock
 */
import model.evaluation.Histogram;
import model.utils.*;

public abstract class AbstractEncoder {
    
    public abstract void set_params(EncoderParams params);
    
    public abstract void init();
    
    /**
     * @brief calculate firing distribution from spike train
     * @param par_spike_train
     * @return distribution (last element for p(none-fire)
     */
    public static double[] spikes2distr(int [][] par_spike_train){
        
        int n = par_spike_train.length;
        if(n < 1){
            return null;
        }
        int _T = par_spike_train[0].length;
        int [][] result = new int[n][_T];
        
        Histogram spiking_hist_layerF = new Histogram();
        spiking_hist_layerF.setParams(n+1); // +1 when no F fires
        spiking_hist_layerF.init();
        for(int ft=0; ft<_T; ++ft){

            int[] spikes_at_T = ModelUtils.extractColumns(par_spike_train, ft);
            boolean bno_f_fired = true;
            for(int i=0; i<n; ++i){

                if(spikes_at_T[i]> 0){
                    spiking_hist_layerF.increment(i);
                    bno_f_fired = false;
                }
            }
            if(bno_f_fired){

                spiking_hist_layerF.increment(n);
            }
        }
        return spiking_hist_layerF.normalize();
    }
    
    /**
     * @brief sample spike train from firing distribution (n nodes + p(none-fires)
     * @param par_distribution
     * @param nof_samples to generate
     * @return spike train
     */
    public static int[][] distr2spikes(double[] par_distribution, int nof_samples){
        
        int n = par_distribution.length-1;
        int [][] spike_train = new int[n][nof_samples];
        Distribution1D spiking_distr_layerF = new Distribution1D();
        spiking_distr_layerF.setParams(par_distribution.length);
        spiking_distr_layerF.init();
        spiking_distr_layerF.evalDistr(par_distribution);

        int nof_samples_to_merge = 1; // produce at least 1 sample
        for(int t=0; t < nof_samples; ++t){

            int [] samples = spiking_distr_layerF.sample(nof_samples_to_merge);
            for(int sample_i=0; sample_i<nof_samples_to_merge; sample_i++){

                int ft_spiking_node = samples[sample_i];
                if(ft_spiking_node < n){
                    spike_train[ft_spiking_node][t] = 1;
                }
            }
        }
        return spike_train;
    }
    
    public static double comp_spikes(int [][] par_spike_train_A, int [][] par_spike_train_B){
        
        double [] distr_A = AbstractEncoder.spikes2distr(par_spike_train_A);
        double [] distr_B = AbstractEncoder.spikes2distr(par_spike_train_B);
        
        double sum_abs_diff = 0;
        for(int i=0; i<distr_A.length; ++i){
            sum_abs_diff += Math.abs(distr_A[i]-distr_B[i]);
        }
        double avg_abs_diff = sum_abs_diff/(double)distr_A.length;
        //System.out.println(java.util.Arrays.toString(distr_A));
        //System.out.println(java.util.Arrays.toString(distr_B));
        return avg_abs_diff;
    }
    
}
