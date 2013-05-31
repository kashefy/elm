/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.simulation;

/**
 *
 * @author woodstock
 */
import model.utils.AbstractParams;
import model.competition.CompetitionParams;
import model.neuron.LearnerParams;
import model.features.FeatureMapParams;
import model.attention.AttentionParams;
import model.attention.SaliencyParams;
import model.utils.files.FileIO;
import org.yaml.snakeyaml.Yaml;

import java.io.*;
import java.util.*;

public class SimulationParams extends AbstractParams{
    
    // file I/O
    private String m_strMainOutputDir;
    private String m_strMainInputDir;
    
    // logging
    private boolean m_log_fresh;
    private boolean m_b_log_weight_watch_layerF;
    private boolean m_b_log_weight_watch_layerZ;
    
    
    // data
    private int m_nofTrainStimuli;
    private int m_nofTestStimuli;
    private int m_nofRows;
    private int m_nofCols;
    
    // encoding parameters
    private double m_encFrequency;
    private double m_deltaT;
    private int m_encDurationInMilSec;
    private int m_popCodeFanOut;
    private boolean m_do_intensity;
    private boolean m_do_orient;

    // prediction parameters
    private int m_nofCauses;
    private int m_nofLearners;
    private int m_nofLearners_layerF;
    private boolean m_b_learn_gaps;
    private boolean m_b_load_layerF;
    
    // parameter objects
    private LearnerParams m_learnerParams;
    private LearnerParams m_learnerParams_layerF;
    private CompetitionParams m_competitionParams;
    private CompetitionParams m_competitionParams_layerF;
    private FeatureMapParams m_featParams;
    private SaliencyParams m_saliencyParams;
    private AttentionParams m_attentionParams;
    
    // attention parameters
    private int m_nofAttentions;
    
    // evaluation parameters
    private int m_predictionStatWindowSize;
    private double m_activityMaskLowerThresholdExcl;
    
    /**
     * @brief load parameters from yaml file
     * @param file path 
     */
    private void load_yml(File par_file){

        Yaml yaml = new Yaml();
        try(InputStream input = new FileInputStream(par_file)){

            Map root = (Map) yaml.load(input);

            Map file_io = (Map) root.get("file I/O");
            m_strMainOutputDir = (String) file_io.get("m_strMainOutputDir");
            m_strMainInputDir = (String) file_io.get("m_strMainInputDir");
            m_b_log_weight_watch_layerF = (boolean) file_io.get("m_b_log_weight_watch_layerF");
            m_b_log_weight_watch_layerZ = (boolean) file_io.get("m_b_log_weight_watch_layerZ");

            Map data = (Map) root.get("data");
            m_nofTrainStimuli = (int) data.get("m_nofTrainStimuli");
            m_nofTestStimuli = (int) data.get("m_nofTestStimuli");
            m_nofRows = (int) data.get("m_nofRows");
            m_nofCols = (int) data.get("m_nofCols");

            Map encoding_parameters = (Map) root.get("encoding");
            m_encFrequency = resolve_int_double_node(encoding_parameters.get("m_encFrequency"), m_encFrequency);
            m_deltaT = (double) encoding_parameters.get("m_deltaT");
            m_encDurationInMilSec = (int) encoding_parameters.get("m_encDurationInMilSec");
            m_popCodeFanOut = (int) encoding_parameters.get("m_popCodeFanOut");
            m_do_orient = (boolean) encoding_parameters.get("m_do_orient");
            m_do_intensity = (boolean) encoding_parameters.get("m_do_intensity");

            Map prediction_parameters = (Map) root.get("prediction");
            m_nofCauses = (int) prediction_parameters.get("m_nofCauses");
            m_nofLearners = (int) prediction_parameters.get("m_nofLearners");
            m_nofLearners_layerF = (int) prediction_parameters.get("m_nofLearners_layerF");
            //m_b_learn_gaps = (boolean) prediction_parameters.get("m_b_learn_gaps");
            m_b_load_layerF = (boolean) prediction_parameters.get("m_b_load_layerF");
            
            Map parameter_objects = (Map) root.get("parameter objects");
            Set<String> keys = parameter_objects.keySet();
            for(String key : keys){

                String str_keyValue = (String) parameter_objects.get(key);
                int index = str_keyValue.lastIndexOf('.'); // determine if key value is a file name
                if( index > 0 ){

                    File parameter_file = new File( str_keyValue );
                    String parameter_file_path;
                    if(!parameter_file.isAbsolute()){

                        parameter_file_path = new File( m_strMainInputDir, str_keyValue ).getPath();
                    }
                    else{
                        parameter_file_path = str_keyValue;
                    }
                    AbstractParams param_obj = null;
                    switch(key){

                        case "m_learnerParams":
                            param_obj = m_learnerParams = new LearnerParams();
                            break;
                        case "m_learnerParams_layerF":
                            param_obj = m_learnerParams_layerF = new LearnerParams();
                            break;
                        case "m_competitionParams":
                            param_obj = m_competitionParams = new CompetitionParams();
                            break;
                        case "m_competitionParams_layerF":
                            param_obj = m_competitionParams_layerF = new CompetitionParams();
                            break;
                        case "m_featParams":
                            param_obj = m_featParams = new FeatureMapParams();
                            break;
                        case "m_attentionParams":
                            param_obj = m_attentionParams = new AttentionParams();
                            break;
                        case "m_saliencyParams":
                            param_obj = m_saliencyParams = new SaliencyParams();
                            break;
                        default:
                            System.err.println("Unknown parameter object type: " + key);
                    }
                    if( param_obj != null ){

                        param_obj.load( parameter_file_path );
                    }
                }
                else{
                    System.err.println("Unknown vlaue type for parameter objects::" + key);
                }
            }
            
            Map attention_parameters = (Map) root.get("attention");
            m_nofAttentions = (int) attention_parameters.get("m_nofAttentions");
            
            Map evaluation_parameters = (Map) root.get("evaluation");
            m_predictionStatWindowSize = (int) evaluation_parameters.get("m_predictionStatWindowSize");
            m_activityMaskLowerThresholdExcl = (double) evaluation_parameters.get("m_activityMaskLowerThresholdExcl");
        }
        catch(Exception e){

            System.err.println("Error: " + e.getMessage());
        }
    }
    
    /**
     * @brief load from old text file
     * @param file path
     */
    private void load_txt(File par_file){
        
        try(Scanner scanner = new Scanner(par_file)){

            String strEntryDelimiter = "\n";
            String strFieldDelimiter = "=";
            scanner.useDelimiter(strEntryDelimiter);
            String strLine;
            String [] arrLineElements;
            String strFieldIdentifier;
            String strFieldValue;

            while(scanner.hasNext()){

                strLine = scanner.next();
                strLine = strLine.replaceAll("\r", "");
                arrLineElements = strLine.split(strFieldDelimiter);

                strFieldIdentifier = arrLineElements[0];
                if(arrLineElements.length > 1){

                    strFieldValue = arrLineElements[1];

                    if(strFieldIdentifier.equalsIgnoreCase("m_strMainOutputDir")){

                        m_strMainOutputDir = strFieldValue;
                        if (!m_strMainOutputDir.endsWith(File.separator))
                            m_strMainOutputDir += File.separator;
                    }
                    if(strFieldIdentifier.equalsIgnoreCase("m_strMainInputDir")){

                        m_strMainInputDir = strFieldValue;
                        if (!m_strMainInputDir.endsWith(File.separator))
                            m_strMainInputDir += File.separator;
                    }
                    else if(strFieldIdentifier.equalsIgnoreCase("m_nofTrainStimuli")){

                        m_nofTrainStimuli = Integer.parseInt(strFieldValue);
                    }
                    else if(strFieldIdentifier.equalsIgnoreCase("m_nofTestStimuli")){

                        m_nofTestStimuli = Integer.parseInt(strFieldValue);
                    }
                    else if(strFieldIdentifier.equalsIgnoreCase("m_nofRows")){

                        m_nofRows = Integer.parseInt(strFieldValue);
                    }
                    else if(strFieldIdentifier.equalsIgnoreCase("m_nofCols")){

                        m_nofCols = Integer.parseInt(strFieldValue);
                    }
                    else if(strFieldIdentifier.equalsIgnoreCase("m_encFrequency")){

                        m_encFrequency = Double.parseDouble(strFieldValue);
                    }
                    else if(strFieldIdentifier.equalsIgnoreCase("m_deltaT")){

                        m_deltaT = Double.parseDouble(strFieldValue);
                    }
                    else if(strFieldIdentifier.equalsIgnoreCase("m_encDurationInMilSec")){

                        m_encDurationInMilSec = Integer.parseInt(strFieldValue);
                    }
                    else if(strFieldIdentifier.equalsIgnoreCase("m_popCodeFanOut")){

                        m_popCodeFanOut = Integer.parseInt(strFieldValue);
                    }
                    else if(strFieldIdentifier.equalsIgnoreCase("m_nofCauses")){

                        m_nofCauses = Integer.parseInt(strFieldValue);
                    }
                    else if(strFieldIdentifier.equalsIgnoreCase("m_nofLearners")){

                        m_nofLearners = Integer.parseInt(strFieldValue);
                    }
                    else if(strFieldIdentifier.equalsIgnoreCase("m_nofLearners_layerF")){

                        m_nofLearners_layerF = Integer.parseInt(strFieldValue);
                    }
                    else if(strFieldIdentifier.equalsIgnoreCase("m_learnerParams")){

                        m_learnerParams = new LearnerParams();
                        if(Character.isLetterOrDigit(strFieldValue.charAt(0))){

                            m_learnerParams.load(m_strMainInputDir + strFieldValue);
                        }
                        else
                            m_learnerParams.load(strFieldValue);

                    }                    
                    else if(strFieldIdentifier.equalsIgnoreCase("m_learnerParams_layerF")){

                        m_learnerParams_layerF = new LearnerParams();
                        if(Character.isLetterOrDigit(strFieldValue.charAt(0))){

                            m_learnerParams_layerF.load(m_strMainInputDir + strFieldValue);
                        }
                        else
                            m_learnerParams_layerF.load(strFieldValue);
                    }
                    else if(strFieldIdentifier.equalsIgnoreCase("m_competitionParams")){

                        m_competitionParams = new CompetitionParams();
                        if(Character.isLetterOrDigit(strFieldValue.charAt(0))){

                            m_competitionParams.load(m_strMainInputDir + strFieldValue);
                        }
                        else
                            m_competitionParams.load(strFieldValue);
                    }
                    else if(strFieldIdentifier.equalsIgnoreCase("m_featParams")){

                        m_featParams = new FeatureMapParams();
                        if(Character.isLetterOrDigit(strFieldValue.charAt(0))){

                            m_featParams.load(m_strMainInputDir + strFieldValue);
                        }
                        else
                            m_featParams.load(strFieldValue);
                    }
                    else if(strFieldIdentifier.equalsIgnoreCase("m_predictionStatWindowSize")){

                        m_predictionStatWindowSize = Integer.parseInt(strFieldValue);
                    }
                    else if(strFieldIdentifier.equalsIgnoreCase("m_nofAttentions")){

                        m_nofAttentions = Integer.parseInt(strFieldValue);
                    }
                    else if(strFieldIdentifier.equalsIgnoreCase("m_activityMaskLowerThresholdExcl")){

                        m_activityMaskLowerThresholdExcl = Double.parseDouble(strFieldValue);
                    }
                    else if(strFieldIdentifier.equalsIgnoreCase("m_attentionParams")){

                        m_attentionParams = new AttentionParams();
                        if(Character.isLetterOrDigit(strFieldValue.charAt(0))){

                            m_attentionParams.load(m_strMainInputDir + strFieldValue);
                        }
                        else
                            m_attentionParams.load(strFieldValue);
                    }
                    else if(strFieldIdentifier.equalsIgnoreCase("m_saliencyParams")){

                        m_saliencyParams = new SaliencyParams();
                        if(Character.isLetterOrDigit(strFieldValue.charAt(0))){

                            m_saliencyParams.load(m_strMainInputDir + strFieldValue);
                        }
                        else
                            m_saliencyParams.load(strFieldValue);
                    }
                }
            }
            scanner.close();
        }
        catch(Exception e){

            System.err.println("Error: " + e.getMessage());
        }
    }
    
    @Override
    public void load(String par_strFilename){
        
        File file = new File(par_strFilename);
        if(FileIO.extension( file.getPath()).equalsIgnoreCase("yml") ){
            
            load_yml(file);
        }
        else{
            load_txt(file);
        }
    }

    public boolean is_log_weight_watch_layerF() {
        return m_b_log_weight_watch_layerF;
    }

    public void set_do_log_weight_watch_layerF(boolean par_b_log_weight_watch_layerF) {
        this.m_b_log_weight_watch_layerF = par_b_log_weight_watch_layerF;
    }

    public boolean is_log_weight_watch_layerZ() {
        return m_b_log_weight_watch_layerZ;
    }

    public void set_do_log_weight_watch_layerZ(boolean par_b_log_weight_watch_layerZ) {
        this.m_b_log_weight_watch_layerZ = par_b_log_weight_watch_layerZ;
    }
    
    public boolean get_log_append() {
        return m_log_fresh;
    }

    public void set_log_append(boolean par_log_fresh) {
        m_log_fresh = par_log_fresh;
    }
    
    public CompetitionParams getCompetitionParamsRef() {
        return m_competitionParams;
    }

    public void setCompetitionParamsRef(CompetitionParams m_competitionParams) {
        this.m_competitionParams = m_competitionParams;
    }

    public CompetitionParams getCompetitionParams_layerF_ref() {
        return m_competitionParams_layerF;
    }

    public void setCompetitionParams_layerF_ref(CompetitionParams m_competitionParams) {
        this.m_competitionParams_layerF = m_competitionParams;
    }
    
    public SaliencyParams getSaliencyParamsRef() {
        return m_saliencyParams;
    }

    public void setSaliencyParamsRef(SaliencyParams m_saliencyParams) {
        this.m_saliencyParams = m_saliencyParams;
    }
    
    public int getNofAttentions() {
        return m_nofAttentions;
    }

    public void setNofAttentions(int m_nofAttentions) {
        this.m_nofAttentions = m_nofAttentions;
    }
    
    public AttentionParams getAttentionParamsRef() {
        return m_attentionParams;
    }

    public void setAttentionParamsRef(AttentionParams m_attentionParams) {
        this.m_attentionParams = m_attentionParams;
    }
    
    public int get_predictionStatWindowSize() {
        return m_predictionStatWindowSize;
    }

    public void set_predictionStatWindowSize(int m_predictionStatWindowSize) {
        this.m_predictionStatWindowSize = m_predictionStatWindowSize;
    }

    public LearnerParams getLearnerParamsRef() {
        return m_learnerParams;
    }

    public void setLearnerParamsRef(LearnerParams m_learnerParams) {
        this.m_learnerParams = m_learnerParams;
    }  
    
    public LearnerParams getLearnerParams_layerF_Ref() {
        return m_learnerParams_layerF;
    }

    public void setLearnerParams_layerF_Ref(LearnerParams m_learnerParams_layerF) {
        this.m_learnerParams_layerF = m_learnerParams_layerF;
    }  
    
    public FeatureMapParams getFeatureMapParamsRef() {
        return m_featParams;
    }

    public void setFeatureMapParamsRef(FeatureMapParams par_featParams) {
        this.m_featParams = par_featParams;
    }  

    public double getDeltaT() {
        return m_deltaT;
    }

    public void setDeltaT(double m_deltaT) {
        this.m_deltaT = m_deltaT;
    }

    public int getEncDurationInMilSec() {
        return m_encDurationInMilSec;
    }

    public void setEncDurationInMilSec(int m_encDurationInMilSec) {
        this.m_encDurationInMilSec = m_encDurationInMilSec;
    }

    public double getEncFrequency() {
        return m_encFrequency;
    }

    public void setEncFrequency(double m_encFrequency) {
        this.m_encFrequency = m_encFrequency;
    }

    public int getNofCauses() {
        return m_nofCauses;
    }

    public void setNofCauses(int m_nofCauses) {
        this.m_nofCauses = m_nofCauses;
    }

    public int getNofCols() {
        return m_nofCols;
    }

    public void setNofCols(int m_nofCols) {
        this.m_nofCols = m_nofCols;
    }

    public int getNofRows() {
        return m_nofRows;
    }

    public void setNofRows(int m_nofRows) {
        this.m_nofRows = m_nofRows;
    }

    public int getNofTestStimuli() {
        return m_nofTestStimuli;
    }

    public void setNofTestStimuli(int m_nofTestStimuli) {
        this.m_nofTestStimuli = m_nofTestStimuli;
    }

    public int getNofTrainStimuli() {
        return m_nofTrainStimuli;
    }

    public void setNofTrainStimuli(int m_nofTrainStimuli) {
        this.m_nofTrainStimuli = m_nofTrainStimuli;
    }

    public int getNofLeaners() {
        return m_nofLearners;
    }
    
    public int getNofLeaners_layerF() {
        return m_nofLearners_layerF;
    }

    public void setNofLearners(int m_nofZNeurons) {
        this.m_nofLearners = m_nofZNeurons;
    }
    
    public boolean is_load_layerF() {
        return m_b_load_layerF;
    }

    public void set_load_layerF(boolean par_b_load_layerF) {
        this.m_b_load_layerF = par_b_load_layerF;
    }

    public int getPopCodeFanOut() {
        return m_popCodeFanOut;
    }

    public void setPopCodeFanOut(int m_popCodeFanOut) {
        this.m_popCodeFanOut = m_popCodeFanOut;
    }
    
    public double get_activityMaskLowerThresholdExcl() {
        return m_activityMaskLowerThresholdExcl;
    }

    public void set_activityMaskLowerThresholdExcl(double m_activityMaskLowerThresholdExcl) {
        this.m_activityMaskLowerThresholdExcl = m_activityMaskLowerThresholdExcl;
    }

    public String getMainInputDir() {
        return m_strMainInputDir;
    }

    public void setMainInputDir(String m_strMainInputDir) {
        this.m_strMainInputDir = m_strMainInputDir;
    }

    public String getMainOutputDir() {
        return m_strMainOutputDir;
    }

    public void setMainOutputDir(String m_strMainOutputDir) {
        this.m_strMainOutputDir = m_strMainOutputDir;
    }
    
    public boolean get_learn_gaps() {
        return m_b_learn_gaps;
    }

    public void set_learn_gaps(boolean par_b_learn_gaps) {
        this.m_b_learn_gaps = par_b_learn_gaps;
    }

    public boolean is_do_intensity() {
        return m_do_intensity;
    }

    public void set_do_intensity(boolean par_do_intensity) {
        this.m_do_intensity = par_do_intensity;
    }

    public boolean is_do_orient() {
        return m_do_orient;
    }

    public void set_do_orient(boolean par_do_orient) {
        this.m_do_orient = par_do_orient;
    }
    
    public SimulationParams(){
        
    }
}
