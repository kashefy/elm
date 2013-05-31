/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.competition;

/**
 *
 * @author woodstock
 */
import model.utils.AbstractParams;
import model.utils.files.FileIO;
import org.yaml.snakeyaml.Yaml;
import java.io.*;
import java.util.*;


public class CompetitionParams extends AbstractParams{
    
    private double m_dt;        // time resolution
    
    private double m_inhibitionAmplitude;    // Amplitude
    private double m_inhibitionTau;         // Time constant of inhibition
    private double m_inhibitonOffset;       // Offset of inhibition

    // Prameters for Ornstein-Uhlenbeck (OU) process:
    
    private double m_ouTau;     // Time constnt of OU process
    private double m_ouSigma;   // Variability of OU process
    private double m_ouMu;      // Offset of OU process
    
    // Parameters for Poisson rate
    private double m_maxRate;
    private double m_maxRateFactor;


    public double get_dt() {
        return m_dt;
    }

    public double getInhibitionTau() {
        return m_inhibitionTau;
    }

    public double getInhibitonOffset() {
        return m_inhibitonOffset;
    }

    public double getInhibitionAmplitude() {
        return m_inhibitionAmplitude;
    }

    public double getOUMu() {
        return m_ouMu;
    }

    public double getOUSigma() {
        return m_ouSigma;
    }

    public double getOUTau() {
        return m_ouTau;
    }
    
    public int set_dt(double par_dt){
        
        if(par_dt < 0)
            return 1;
        else
            m_dt = par_dt;
        return 0;
    }
    
    public double getMaxRate() {
        return m_maxRate;
    }
    
    public int setMaxRate(double par_maxRate){
  
        if(par_maxRate < 0)
            return 1;
        else
            m_maxRate = par_maxRate;
        return 0;
    }
    
    public double getMaxRateFactor() {
        return m_maxRateFactor;
    }
    
    public void setMaxRateFactor(double par_maxRateFactor) {
        
        m_maxRateFactor = par_maxRateFactor;
        setMaxRate(1.0/(m_maxRateFactor * m_dt));
    }
    
    public void calcMaxRateFromFactor(double par_maxRateFactor) {
        
        m_maxRateFactor = par_maxRateFactor;
        setMaxRate(1./(m_maxRateFactor * m_dt));
    }
    
    public void setInhibitionParams(double par_inhibtionAmplitude,
            double par_inhibitionTau,
            double par_inhibitonOffset){
        
            m_inhibitionAmplitude    = par_inhibtionAmplitude;
            m_inhibitionTau         = par_inhibitionTau;     
            m_inhibitonOffset       = par_inhibitonOffset;     
    }
    
    public void setOUParams(double par_ouTau,
        double par_ouSigma,
        double par_ouMu){
        
        m_ouTau     = par_ouTau;  
        m_ouSigma   = par_ouSigma;
        m_ouMu      = par_ouMu;   
    }   
    
    private void load_yml(File par_file){

        Yaml yaml = new Yaml();
        try(InputStream input = new FileInputStream(par_file)){

            Map root = (Map) yaml.load(input);
                
            m_dt = (double) root.get("m_dt");
            m_inhibitionAmplitude = (double) root.get("m_inhibitionAmplitude");
            m_inhibitionTau = (double) root.get("m_inhibitionTau");
            m_inhibitonOffset = (double) root.get("m_inhibitonOffset");
            m_ouTau = (double) root.get("m_ouTau");
            m_ouSigma = (double) root.get("m_ouSigma"); 
            m_ouMu = (double) root.get("m_ouMu");
            if( root.containsKey("m_maxRate") ){

                // function of dt, set during runtime
                m_maxRate = (double) root.get("m_maxRate");
            }
            else if( root.containsKey("m_maxRateFactor") ){
                
                Object node = root.get("m_maxRateFactor");
                m_maxRateFactor = resolve_int_double_node(node, m_maxRateFactor);
                calcMaxRateFromFactor(m_maxRateFactor);  
            }
        }
        catch(Exception e){

            System.err.println("Error: " + e.getMessage());
        }
    }
    
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
                strFieldValue = arrLineElements[1];
                if(strFieldIdentifier.equalsIgnoreCase("m_dt")){
                    
                    m_dt = Double.parseDouble(strFieldValue);
                }
                if(strFieldIdentifier.equalsIgnoreCase("m_inhibitionAmplitude")){
                    
                    m_inhibitionAmplitude = Double.parseDouble(strFieldValue);
                }
                else if(strFieldIdentifier.equalsIgnoreCase("m_inhibitionTau")){
                    
                    m_inhibitionTau = Double.parseDouble(strFieldValue);
                }
                else if(strFieldIdentifier.equalsIgnoreCase("m_inhibitonOffset")){
                    
                    m_inhibitonOffset = Double.parseDouble(strFieldValue);
                }
                else if(strFieldIdentifier.equalsIgnoreCase("m_ouTau")){
                    
                    m_ouTau = Double.parseDouble(strFieldValue);
                }   
                else if(strFieldIdentifier.equalsIgnoreCase("m_ouSigma")){
                    
                    m_ouSigma = Double.parseDouble(strFieldValue);
                }   
                else if(strFieldIdentifier.equalsIgnoreCase("m_ouMu")){
                    
                    m_ouMu = Double.parseDouble(strFieldValue);
                }                
                else if(strFieldIdentifier.equalsIgnoreCase("m_maxRate")){
                    
                    m_maxRate = Double.parseDouble(strFieldValue);
                    // function of dt, set during runtime
                }
                else if(strFieldIdentifier.equalsIgnoreCase("m_maxRateFactor")){
                    
                    m_maxRateFactor = Double.parseDouble(strFieldValue);
                    calcMaxRateFromFactor(m_maxRateFactor);
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
}
