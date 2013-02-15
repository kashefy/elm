/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.utils.files;

/**
 *
 * @author woodstock
 */
import model.utils.files.FileIO;

public class DataLoaderTest {
    
    public static void testTranslation(){
 
        DataLoaderImageSetCSV dataLoader;
        String strDataDir = ".\\data\\input\\MNIST";

        dataLoader = new DataLoaderImageSetCSVTranslation();
        dataLoader.setParams(strDataDir);
        dataLoader.init();
        dataLoader.load();
        
        int [] arrDims  = dataLoader.getDims();
        int nofRows = arrDims[ FileIO.DIM_INDEX_ROWS ];
        int nofCols = arrDims[ FileIO.DIM_INDEX_COLS ];
        
        System.out.println("nofRows: "+nofRows);
        System.out.println("nofCols: "+nofCols);
        
        int nofSamples = dataLoader.getNofSamples();
        nofSamples = 1;
        
        String strOut = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\output\\MNIST\\";
        
        for(int i=0; i<nofSamples; i++){
            
            // display label
            //System.out.print("l"+i+":"+dataLoader.getLabel(i));
            System.out.println();
            
            // display sample
            double [] sample = dataLoader.getSample(33);     
//            for(int r=0; r<nofRows; r++){
//                
//                int rowIndex = r*nofCols;
//                for(int c=0; c<nofCols; c++){
//                    
//                    int binVal = (int)sample[rowIndex+c];
//                    System.out.print(binVal);
//                }
//                System.out.println();    
//            }
            
            FileIO.saveArrayToCSV(sample, nofRows, nofCols, strOut+"scene"+i+".csv");
            
        }      

        
    }
    
    public static void testLoadMNISTCSV(){
    
        DataLoaderImageSetCSV dataLoader;
        String strDataDir = ".\\data\\input\\MNIST";

        dataLoader = new DataLoaderImageSetCSV();
        dataLoader.setParams(strDataDir);
        dataLoader.init();
        dataLoader.load();
        
        int [] arrDims  = dataLoader.getDims();
        int nofRows = arrDims[ FileIO.DIM_INDEX_ROWS ];
        int nofCols = arrDims[ FileIO.DIM_INDEX_COLS ];
        
        System.out.println("nofRows: "+nofRows);
        System.out.println("nofCols: "+nofCols);
        
        int nofSamples = dataLoader.getNofSamples();
        
        String strOut = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\output\\MNIST\\";
        
        
        for(int i=0; i<nofSamples; i++){
            
            // display label
            //System.out.print("l"+i+":"+dataLoader.getLabel(i));
            System.out.println();
            
            // display sample
            double [] sample = dataLoader.getSample(i);     
            for(int r=0; r<nofRows; r++){
                
                int rowIndex = r*nofCols;
                for(int c=0; c<nofCols; c++){
                    
                    int binVal = (int)sample[rowIndex+c];
                    System.out.print(binVal);
                }
                System.out.println();    
            }
            
            FileIO.saveArrayToCSV(sample, nofRows, nofCols, strOut+i+".csv");
            
        }      
            
    }
    
}
