/*
 *  HOUGH_D_RT Helper function for Hough Transform block.
 *
 *  Copyright 1995-2004 The MathWorks, Inc.
 */
#include "viphough_rt.h"  

LIBMWVISIONRT_API void MWVIP_Hough_D(
    const boolean_T  *uBW,
    real_T           *yH,
    const real_T     *sineTablePtr, 
    const real_T     *rho,
    int_T inRows,
    int_T inCols,
    int_T rhoLen,
    int_T Ceil90ByThResPlus1
)
{
    int_T thetaLen = 2*Ceil90ByThResPlus1-2;    
    real_T firstRho = rho[0];
    real_T slope = ((firstRho==0) && (rhoLen==1))
        ? 0 : (rhoLen - 1)/(-2*firstRho) ; /* (endRho - firstRho), endRho=rho[rhoLen-1]=-firstRho); */
    int_T n,m,thetaIdx,j;
    int_T rhoIdx;

    memset((byte_T *)yH,0,rhoLen*thetaLen*sizeof(real_T));
    
    /* Compute the hough transform */
    for(n=0; n < inCols; n++)
    {
        int_T saved_idx;
        for(m=0; m < inRows; m++)
        {
            if(uBW[n*inRows+m]) /* if pixel is 1 (on) */
            {
                saved_idx = Ceil90ByThResPlus1-1;
                for(thetaIdx=0; thetaIdx<Ceil90ByThResPlus1; thetaIdx++)
                {   /* theta varies from -90 to 0 */
                    /* x*cos(theta)+y*sin(theta)=rho */
                    real_T myrho = n*(-sineTablePtr[saved_idx-thetaIdx])+m*sineTablePtr[thetaIdx];
                    real_T tmpRhoIdx = slope*(myrho - firstRho);
                    /* convert to bin index */
                    rhoIdx = (tmpRhoIdx>0)? (int_T)(tmpRhoIdx+0.5): (int_T)(tmpRhoIdx-0.5); 
                    yH[thetaIdx*rhoLen+rhoIdx]++; /* increment counter */
                }
                saved_idx = Ceil90ByThResPlus1-2;
                for( j=0; thetaIdx<thetaLen; thetaIdx++,j++)
                {
                    /* x*cos(theta)+y*sin(theta)=rho */
                    real_T myrho = n*(-sineTablePtr[j+1])+m*(-sineTablePtr[saved_idx-j]); 
                    real_T tmpRhoIdx = slope*(myrho - firstRho);
                    /* convert to bin index */
                    rhoIdx = (tmpRhoIdx>0)? (int_T)(tmpRhoIdx+0.5): (int_T)(tmpRhoIdx-0.5); 
                    yH[thetaIdx*rhoLen+rhoIdx]++; /* increment counter */
                }
            }
        }
    }
}

/* [EOF] skew_bci_d_rt.c */
