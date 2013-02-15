/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.utils.files;

/**
 *
 * @author woodstock
 */
public abstract class AbstractDataLoader {
    
    protected String m_strDataPath;
    protected int m_nofSamples;
    protected int[] m_arrSampleDims;
    protected int m_nofClasses;
    
    public abstract int setParams(String par_strDataDir);

    public abstract void init();

    public abstract void load();

    public int [] getDims(){
        
        int nofDims = m_arrSampleDims.length;
        int [] dimsToExport = new int [ nofDims ];
        System.arraycopy(m_arrSampleDims, 0, dimsToExport, 0, nofDims);
        return dimsToExport;
        
    }

    public int getNofSamples(){

        return m_nofSamples;
    }
        
    public int getNofClasses(){
        
        return m_nofClasses;
    }

    public abstract double [] getSample(int par_index);  

    public abstract int getLabel(int par_index);

    /**
     * @brief determine classes represented in subset. Error message if not all classes are present.
     * @param starti
     * @param endi
     * @return classes represented
     */
    public abstract int[] determineLabelSet(int starti, int endi);
    
}
