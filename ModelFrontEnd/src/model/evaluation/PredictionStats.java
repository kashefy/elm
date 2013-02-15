/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.evaluation;

/**
 *
 * @author woodstock
 */
import model.competition.AbstractCompetition;
import java.util.*;

public class PredictionStats {
    
    public static int RESPONSE_ALL = AbstractCompetition.WTA_ALL;
    
    private int m_nofLearners;
    private int m_nofCauses;
    private int [] m_arrLabelNames;

    private Histogram[] m_histograms;
    private double [][] m_arrFiringProb;
    private boolean m_bCalcsUpToDate;
    
    private int m_windowSize;
    private ArrayList m_arr_results_avg_cond_entropy;
    private int m_response_count;

    public double[] get_results_avg_cond_entropy() {
        
        int size = m_arr_results_avg_cond_entropy.size();
        Double [] temp = (Double[]) m_arr_results_avg_cond_entropy.toArray(new Double[ size ]);
        double [] temp_primitive = new double[ size ];
        for(int i=0; i<size; i++){
            temp_primitive[i]=temp[i].doubleValue();
        }
        return temp_primitive;
    }
    
    public void setParams(EvaluationParams params){
        
        if(params.m_nofLearners > 0){
            
            m_nofLearners = params.m_nofLearners;
        }
        m_nofCauses = params.m_arr_label_names.length;
        m_arrLabelNames = params.get_label_names();
        m_windowSize = params.m_predictionStats_windowSize;
    }
        
    public void setParams(int par_nofLearners, int [] par_arrLabelNames){
        
        if(par_nofLearners > 0){
            
            m_nofLearners = par_nofLearners;
        }
        m_nofCauses = par_arrLabelNames.length;
        m_arrLabelNames = new int [ m_nofCauses ];
        System.arraycopy(par_arrLabelNames, 0, m_arrLabelNames, 0, m_nofCauses);
    }
    
    public void init(){
        
        m_histograms = new Histogram[ m_nofLearners ];
        for(int i=0; i<m_nofLearners; i++){
            
            m_histograms[i] = new Histogram();
            m_histograms[i].setParams(m_nofCauses);
            m_histograms[i].init();
        }
        m_bCalcsUpToDate = false;
        
        if(m_windowSize > 0){
            
            m_arr_results_avg_cond_entropy = new ArrayList(m_windowSize);
            m_response_count = 0;
        }
    }
    
    public void addResponse(int par_learnerIndex, int par_labelName){

        int label_name_index = Arrays.binarySearch(m_arrLabelNames, par_labelName);
        boolean bFound = label_name_index >= 0;
        if(bFound){
            
            if(par_learnerIndex == RESPONSE_ALL){

                for(int li=0; li<m_nofLearners; li++){
                    
                    m_histograms[ li ].increment(label_name_index);
                }
            }
            else{

                m_histograms[ par_learnerIndex ].increment(label_name_index);
            }   
            m_bCalcsUpToDate = false;
            
            if(m_windowSize > 0){// implicit calculation and resetting
            
                m_response_count++;
                if(m_response_count % m_windowSize == 0){
                    
                    m_arr_results_avg_cond_entropy.add(calcConditionalEntropy());
                    reset();
                }
            }
        }
    }
    
    public double [][] calcFiringProbs(){
        
        m_arrFiringProb = new double [ m_nofLearners ][ m_nofCauses ];
        double [][] arrFiringProbToExport = new double [ m_nofLearners ][ m_nofCauses ];
        for(int i=0; i<m_nofLearners; i++){

            m_arrFiringProb[ i ] = m_histograms[ i ].normalize();
            
            System.arraycopy(m_arrFiringProb[i], 0, arrFiringProbToExport[i], 0, m_nofCauses);
        }
        m_bCalcsUpToDate = true;
        return arrFiringProbToExport;
    }
    
    public int[][] getFiringCounts(){
        
        int [][] arrFiringCountsToExport = new int [ m_nofLearners ][ m_nofCauses ];
        for(int i=0; i<m_nofLearners; i++){

            arrFiringCountsToExport[i] = m_histograms[i].getCounts();
        }
        return arrFiringCountsToExport;        
    }
    
    public int[] calcFiringSums(){
        
        int [] arrFiringSumsToExport = new int [ m_nofLearners ];
        for(int i=0; i<m_nofLearners; i++){

            arrFiringSumsToExport[i] = m_histograms[i].calcSum();
        }
        return arrFiringSumsToExport;        
    }
    
    public double calcFiringSums(int[] par_arrVvalueRef){
        
        int sum = 0;
        for(int i=0; i<m_nofLearners; i++){

            int val = m_histograms[i].calcSum();
            par_arrVvalueRef[i] = val;
            sum += val;
        }
        double avg = sum/(double)(m_nofLearners);
        return avg;        
    }
    
    /**
     * 
     * @return max values for each histograms per learner
     */
    public int [] findMax(){
        
        int [] arrMax = new int [ m_nofLearners ]; 
        for(int i=0; i<m_nofLearners; i++){
            
            arrMax[i] = m_histograms[i].findMax();
        }
        return arrMax;
    }
    
    /**
     * fills out conditional entropy values of the individual learners in the passed array reference
     * 
     * @param [out]par_arrVvalueRef
     * @return return average conditional entropy over all learners
     */
    public double calcConditionalEntropy(double [] par_arrVvalueRef){
        
        if(!m_bCalcsUpToDate){
            this.calcFiringProbs();
        }
        double sum = 0;
        for(int i=0; i<m_nofLearners; i++){
            
            double value = ConditionalEntropy.calculate(m_arrFiringProb[i]);
            par_arrVvalueRef[i] = value;
            sum += value;
        }
        double avg = sum/(double)m_nofLearners;
        //System.out.println(avg);
        return avg;
    }
    
    public double calcConditionalEntropy(){
        
        double [] arrValueRef = new double[ m_nofLearners ];
        return calcConditionalEntropy(arrValueRef);
    }
     
    public void reset(){
        
        for(int li=0; li<m_nofLearners; li++){
            
            m_histograms[li].reset();
        }
        m_arrFiringProb = new double [ m_nofLearners ][ m_nofCauses ];
        m_bCalcsUpToDate = false;
        m_response_count = 0;
    }
    
    public int[] get_label_names() {
        
        int [] arr_label_names_2_export = new int [ m_arrLabelNames.length ];
        System.arraycopy(m_arrLabelNames, 0, arr_label_names_2_export, 0, m_arrLabelNames.length);
        return arr_label_names_2_export;
    }
        
    public PredictionStats(){
        
    }
}
