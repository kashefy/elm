/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.attention;

/**
 *
 * @author woodstock
 */
import model.utils.AbstractParams;
import model.utils.files.FileIO;
import org.yaml.snakeyaml.Yaml;
import java.io.*;
import java.util.*;

public class AttentionParams extends AbstractParams{
    
    protected String m_strMainInputDir;
    protected SaliencyParams m_saliencyParams;
    protected int m_nofWindowRows;
    protected int m_nofWindowCols;
    
    private void load_yml(File par_file){

        Yaml yaml = new Yaml();
        try(InputStream input = new FileInputStream(par_file)){

            Map root = (Map) yaml.load(input);
                
            Map file_io = (Map) root.get("file I/O");
            m_strMainInputDir = (String) file_io.get("m_strMainInputDir");

            Map data = (Map) root.get("data");
            m_nofWindowRows = (int) data.get("m_nofWindowRows");
            m_nofWindowCols = (int) data.get("m_nofWindowCols");
            
            Map parameter_objects = (Map) root.get("parameter objects");
            Set<String> keys = parameter_objects.keySet();
            for(String key : keys){

                String str_keyValue = (String) parameter_objects.get(key);
                int index = str_keyValue.lastIndexOf('.'); // determine if key value is a file name
                if( index > 0 ){

                    File parameter_file = new File( str_keyValue );
                    String parameter_file_path;
                    if(! parameter_file.isAbsolute() ){

                        parameter_file_path = new File( m_strMainInputDir, str_keyValue ).getPath();
                    }
                    else{
                        parameter_file_path = str_keyValue;
                    }
                    AbstractParams param_obj = null;
                    switch(key){

                        case "m_saliencyParams":
                            param_obj = m_saliencyParams = new SaliencyParams();
                            break;
                        default:
                            System.err.println("Unknown parameter object type: "+key);
                    }
                    if( param_obj != null ){

                        param_obj.load( parameter_file_path );
                    }
                }
                else{
                    System.err.println("Unknown vlaue type for parameter objects::"+key);
                }
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
                if (!strLine.startsWith("//")){
                    arrLineElements = strLine.split(strFieldDelimiter);

                    strFieldIdentifier = arrLineElements[0];
                    strFieldValue = arrLineElements[1];

                    if(strFieldIdentifier.equalsIgnoreCase("m_strMainInputDir")){

                            m_strMainInputDir = strFieldValue;
                            if (!m_strMainInputDir.endsWith(File.separator))
                                m_strMainInputDir += File.separator;
                        }
                    else if(strFieldIdentifier.equalsIgnoreCase("m_nofWindowRows")){

                        m_nofWindowRows = Integer.parseInt(strFieldValue);
                    }
                    else if(strFieldIdentifier.equalsIgnoreCase("m_nofWindowCols")){

                        m_nofWindowCols = Integer.parseInt(strFieldValue);
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
    
    @Override
    public void save(String par_str_filepath){
        
        Yaml yaml = new Yaml();
        Map<String, Object> root = new HashMap<>();
        try(PrintWriter p = new PrintWriter(par_str_filepath)){

            Map<String, Object> file_io = new HashMap<>();
            file_io.put("m_strMainInputDir", m_strMainInputDir);
            root.put("file I/O", file_io);
            
            Map<String, Object> data = new HashMap<>();
            data.put("m_nofWindowRows", m_nofWindowRows);
            data.put("m_nofWindowCols", m_nofWindowCols);
            root.put("data", data);
            
            Map<String, Object> parameter_objects = new HashMap<>();
            String filename_params;
            filename_params = "saliencyParamFile.yml";
            parameter_objects.put("m_saliencyParams", filename_params);
            //m_saliencyParams.save(filename_params);
            root.put("parameter objects", parameter_objects);
            
            p.println(yaml.dump(root));
            p.close();
        }
        catch(Exception e){
            System.err.println("Error: " + e.getMessage());
        }
    }
    
    public int get_nofWindowCols() {
        return m_nofWindowCols;
    }

    public int get_nofWindowRows() {
        return m_nofWindowRows;
    }

    public SaliencyParams get_saliencyParamsRef() {
        return m_saliencyParams;
    }
    
    public AttentionParams(){
        
    }
}
