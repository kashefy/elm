/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.evaluation;

/**
 *
 * @author woodstock
 */
import java.util.HashSet;
import java.util.Random;
import java.util.Set;
import java.util.Iterator;

public class EvaluationTest {
    
    /**
     * test window feature in prediction stats class
     * @return 0 on success, non-zero for error
     */
    public static int test_predictionStats_window(){
        
        int nof_learners = 4;
        int nof_causes   = 4;
        int window_size  = 10;
        int nof_trials = 1000;
                
        int [] label_names = new int[ nof_causes ];
        emulate_label_names(label_names);
        
        EvaluationParams evaluation_params = new EvaluationParams();
        evaluation_params.set_nofLearners(nof_learners);
        evaluation_params.set_label_names(label_names);
        evaluation_params.set_predictionStats_windowSize(window_size);
        
        PredictionStats prediction_stats = new PredictionStats();
        prediction_stats.setParams(evaluation_params);
        prediction_stats.init();
        
        Random generator = new Random();
        for(int i=0; i<nof_trials; i++){
            
            int learner_index;
            int label_index;
            
            learner_index = generator.nextInt(nof_learners);
            label_index = generator.nextInt(nof_causes);
            
            int label = label_names[ label_index ];
            prediction_stats.addResponse(learner_index, label);
        }
        
        double [] results_avg_cond_entropy = prediction_stats.get_results_avg_cond_entropy();
        
        return 0;
    }
    
    /**
     * generate random label names
     * @param [out]par_label_names_ref 
     */
    public static void emulate_label_names(int[] par_label_names_ref){
        
        int nof_causes = par_label_names_ref.length;
        Random generator = new Random();
        int causes_range = nof_causes*10;
        // generate label names
        Set<Integer> label_names_set = new HashSet<>();
        while(label_names_set.size() < nof_causes){
            
            int entry = generator.nextInt(causes_range);
            label_names_set.add(entry);
        }
        
        Iterator it = label_names_set.iterator();
        int i=0;
        while(it.hasNext()){
            
            Integer entry = (Integer)it.next();
            par_label_names_ref[i++] = entry.intValue();
        }
    }
}
