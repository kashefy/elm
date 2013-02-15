/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.evaluation;

/**
 *
 * @author woodstock
 */
public class EvaluationParams {
    
    protected int m_nofLearners;
    protected int [] m_arr_label_names;
    protected int m_predictionStats_windowSize;

    public int [] get_label_names() {
        
        int size = m_arr_label_names.length;
        int [] to_export = new int[ size ];
        System.arraycopy(m_arr_label_names, 0, to_export, 0, size);
        return to_export;
    }

    public void set_label_names(int [] par_arr_label_names) {
        
        int size = par_arr_label_names.length;
        m_arr_label_names = new int[ size ];
        System.arraycopy(par_arr_label_names, 0, m_arr_label_names, 0, size);
    }

    public int get_nofLearners() {
        return m_nofLearners;
    }

    public void set_nofLearners(int par_nofLearners) {
        this.m_nofLearners = par_nofLearners;
    }

    public int get_predictionStats_windowSize() {
        return m_predictionStats_windowSize;
    }

    public void set_predictionStats_windowSize(int par_predictionStats_windowSize) {
        this.m_predictionStats_windowSize = par_predictionStats_windowSize;
    }
    
    public EvaluationParams(){
        
    }
}
