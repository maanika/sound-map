/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Diagnostics;
using System.Drawing;
using System.Text;
using System.Windows.Forms;

using CyDesigner.Extensions.Common;
using CyDesigner.Extensions.Gde;

// The namespace is required to have the same name as the component for a customizer.
namespace DDS24_v0_0
{
    
    class CyCustomizer : ICyShapeCustomize_v1
    {
        const string OUTPUT_TERM_NAME_1 = "out1"; //out1 terminal
        const string OUTPUT_TERM_NAME_2 = "out2"; //out2 terminal


        public CyCustErr CustomizeShapes(ICyInstQuery_v1 instQuery, ICySymbolShapeEdit_v1 shapeEdit,
            ICyTerminalEdit_v1 termEdit)
        {
            CyCustErr err;

            // We leave the symbol as it is for symbol preview
            if (instQuery.IsPreviewCanvas)
                return CyCustErr.OK;
            
            //todo: if single output -> name it "out" instead of "out1" 
            
            // Read Parameters
            CyCompDevParam outWidthParam1 = instQuery.GetCommittedParam("out1_width");
            byte outWidth1 = byte.Parse(outWidthParam1.Value);
            byte maxOutBitIndex1 = (byte)(outWidth1 - 1);
            string outTermName1 = termEdit.GetTermName(OUTPUT_TERM_NAME_1);
            
            if (maxOutBitIndex1 != 0)
                err = termEdit.TerminalRename(outTermName1, string.Format("{0}[{1}:0]", OUTPUT_TERM_NAME_1, maxOutBitIndex1.ToString())); 
            else
                err = termEdit.TerminalRename(outTermName1, string.Format("{0}", OUTPUT_TERM_NAME_1 ));   
     
            if (err.IsNotOK) return err; 
            
            
            
            // Read Parameters
            CyCompDevParam outWidthParam2 = instQuery.GetCommittedParam("out2_width");
            byte outWidth2 = byte.Parse(outWidthParam2.Value);
            byte maxOutBitIndex2 = (byte)(outWidth2 - 1);
            string outTermName2 = termEdit.GetTermName(OUTPUT_TERM_NAME_2);
            
            if (maxOutBitIndex2 != 0)
                err = termEdit.TerminalRename(outTermName2, string.Format("{0}[{1}:0]", OUTPUT_TERM_NAME_2, maxOutBitIndex2.ToString())); 
            else
                err = termEdit.TerminalRename(outTermName2, string.Format("{0}", OUTPUT_TERM_NAME_2 ));   
     
            if (err.IsNotOK) return err; 
          
            
            

            return CyCustErr.OK;
        }
    }    
}

//[] END OF FILE
