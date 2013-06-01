/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.encoding;

/**
 *
 * @author woodstock
 */
import model.utils.AbstractParams;
import org.yaml.snakeyaml.Yaml;
import model.utils.files.FileIO;
import model.features.AbstractFeatureMap;
import java.io.*;
import java.util.*;
import java.lang.UnsupportedOperationException;

public class EncoderParams extends AbstractParams{
    
    protected AbstractFeatureMap m_feature_map; // cannot load, needs to be setted
    protected double m_frequency;
    protected double m_delta_t;
    protected int m_duration_milSec;
    protected int m_nof_pop_code_inputs;
    protected int m_pop_code_fan_out;
    protected int [] m_arr_input_dims;
    
    @Override
    public void load(String par_strFilename){
        
        throw new UnsupportedOperationException("Loading from file not supported.");
    }
    
    @Override
    public void save(String par_str_filepath){
        
        throw new UnsupportedOperationException("Saving to file not supported.");
    }

    /**
     * 
     * @return return copy of dimensions, not reference to internal object
     */
    public int[] get_arr_input_dims() {
        
        int size = m_arr_input_dims.length;
        int [] arr2Export = new int[size];
        System.arraycopy(m_arr_input_dims, 0, arr2Export, 0, size);
        return arr2Export;
    }

    /**
     * Create own internal copy of dimensions array
     * @param m_arr_input_dims 
     */
    public void set_arr_input_dims(int[] m_arr_input_dims) {
        
        int size = m_arr_input_dims.length;
        this.m_arr_input_dims = new int[size];
        System.arraycopy(m_arr_input_dims, 0, this.m_arr_input_dims, 0, size);
    }

    public double get_delta_t() {
        return m_delta_t;
    }

    public void set_delta_t(double m_delta_t) {
        this.m_delta_t = m_delta_t;
    }

    public int get_duration_milSec() {
        return m_duration_milSec;
    }

    public void set_duration_milSec(int m_duration_milSec) {
        this.m_duration_milSec = m_duration_milSec;
    }

    /**
     * @return reference to feature map object
     */
    public AbstractFeatureMap get_feature_map() {
        return m_feature_map;
    }

    /**
     * sets member object to same reference, no re-instantiation
     * @param reference to feature map 
     */
    public void set_feature_map(AbstractFeatureMap m_feature_map) {
        this.m_feature_map = m_feature_map;
    }

    public double get_frequency() {
        return m_frequency;
    }

    public void set_frequency(double m_frequency) {
        this.m_frequency = m_frequency;
    }

    public int get_nof_pop_code_inputs() {
        return m_nof_pop_code_inputs;
    }

    public void set_nof_pop_code_inputs(int m_nof_pop_code_inputs) {
        this.m_nof_pop_code_inputs = m_nof_pop_code_inputs;
    }

    public int get_pop_code_fan_out() {
        return m_pop_code_fan_out;
    }

    public void set_pop_code_fan_out(int pop_code_fan_out) {
        this.m_pop_code_fan_out = pop_code_fan_out;
    }
        
    public EncoderParams(){
        
    }
}
