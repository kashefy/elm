/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.features;

/**
 *
 * @author woodstock
*/
import org.shared.array.ComplexArray;
import model.utils.ModelUtils;
import model.utils.files.FileIO;
import org.shared.array.RealArray;

public class DiffOfGaussians2DSq{
    
    protected int m_supportRadius;
    protected double m_nScale;
    protected double m_sigmaSurr;
    protected double m_sigmaCent;
    protected double m_nBalance;
    protected int m_nCentreOnOff;
    protected RealArray m_kernel;
    
    public void setParams(int par_supportRadius, double par_nScale){
        
        m_supportRadius = par_supportRadius;
        m_nScale = par_nScale;
        
        m_sigmaSurr = 10 * m_nScale;
        m_sigmaCent = 2 * m_nScale;
        
    }
    
    public void init(){
        
        int nofRows = m_supportRadius*2+1;
        int nofCols = nofRows;
        
        double sigmaSSq = m_sigmaSurr * m_sigmaSurr;
        double sigmaCSq = m_sigmaCent * m_sigmaCent;
        
        double [] vals = new double [ nofRows*nofCols ];
        
        int x = -m_supportRadius;        
        for(int r=0; r<nofRows; r++){
            
            int xSq = x*x;
            int y = -m_supportRadius;
            
            int rowOffset = r*nofCols;
            for(int c=0; c<nofCols; c++){
                
                int ySq = y*y;
                int rSq = xSq + ySq;
    
                vals[ rowOffset+c ] = m_nCentreOnOff * 
                        (
                        1/(1*Math.PI*sigmaCSq) * Math.exp(-rSq/(2*sigmaCSq)) - 
                        m_nBalance/(2*Math.PI*sigmaSSq) * Math.exp(-rSq/(2*sigmaSSq))
                        );
                y++;
            }
            x++;
        }
        double [] valsComplex = ModelUtils.real2Complex(vals);
        //int [] dims = new int[]{nofRows, nofCols, 2};
        //m_kernel = new ComplexArray(valsComplex,dims);
        int [] dims = new int[]{nofRows, nofCols};
        m_kernel = new RealArray(vals,dims);
    }
    
    public RealArray getKernelRef(){
        
        return m_kernel;
    }
    
    public double [] getKernelValues(){
        
        int nofRows = m_supportRadius*2+1;
        int nofCols = nofRows;
        int nofElements = nofRows * nofCols * 2; // *2 for complex
        double [] valuesToExport = new double [ nofElements ];
        System.arraycopy(m_kernel.values(), 0, valuesToExport, 0, nofElements);
        return valuesToExport;
    }
    
    public double getSupportRadius() {
        return m_supportRadius;
    }

    public void setSupportRadius(int par_supportRadius) {
        this.m_supportRadius = par_supportRadius;
    }
    
    public double getScale() {
        return m_nScale;
    }

    public void setScale(double par_nScale) {
        this.m_nScale = par_nScale;
    }

    public double getBalance() {
        return m_nBalance;
    }

    public void setBalance(double par_nBalance) {
        this.m_nBalance = par_nBalance;
    }

    public boolean getCentreOnOff() {
        return m_nCentreOnOff == 1;
    }

    public void setCentreOnOff(boolean par_bCentreOnOff) {
        this.m_nCentreOnOff = (par_bCentreOnOff)? 1 : 0;
    }

    public double getSigmaCentre() {
        return m_sigmaCent;
    }

    public void setSigmaCentre(double par_sigmaCent) {
        this.m_sigmaCent = par_sigmaCent;
    }

    public double getSigmaSurround() {
        return m_sigmaSurr;
    }

    public void setSigmaSurround(double par_sigmaSurr) {
        this.m_sigmaSurr = par_sigmaSurr;
    }
    
    public void test(){
        
        int supportRadius = 14;
        double scale = 1;
        this.setParams(supportRadius, scale);
        this.init();
        
        int [] dims = m_kernel.dims();
        int nofRows = dims[ FileIO.DIM_INDEX_ROWS ];
        int nofCols = dims[ FileIO.DIM_INDEX_COLS ];
        //FileIO.saveArrayToCSV(m_kernel.torRe().values(), nofRows, nofCols, "ref.csv");
        //FileIO.saveArrayToCSV(m_kernel.torIm().values(), nofRows, nofCols, "imf.csv");
        FileIO.saveArrayToCSV(m_kernel.values(), nofRows, nofCols, "ref.csv");
        
    }
    
    public DiffOfGaussians2DSq(){
        
        m_nBalance = 5.0;
        m_nCentreOnOff = 1;
    }
}
