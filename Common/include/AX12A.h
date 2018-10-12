/**
  *****************************************************************************
  * @file    AX12A.h
  * @author  Tyler
  *
  * @defgroup AX12A
  * @ingroup Dynamixel
  * @{
  *****************************************************************************
  */




#ifndef AX12A_H
#define AX12A_H




/********************************* Includes **********************************/
#include "Dynamixel.h"




/*********************************** Motor ***********************************/
namespace dynamixel{
// Constants
// ----------------------------------------------------------------------------

// Default register values
// ----------------------------------------------------------------------------
//extern const uint8_t AX12A_DEFAULT_BAUD_RATE;
//extern const uint16_t AX12A_DEFAULT_CCW_ANGLE_LIMIT;
//extern const uint8_t AX12A_DEFAULT_HIGHEST_VOLTAGE_LIMIT;
//extern const uint8_t AX12A_DEFAULT_CW_COMPLIANCE_MARGIN;
//extern const uint8_t AX12A_DEFAULT_CCW_COMPLIANCE_MARGIN;
//extern const uint8_t AX12A_DEFAULT_CW_COMPLIANCE_SLOPE;
//extern const uint8_t AX12A_DEFAULT_CCW_COMPLIANCE_SLOPE;
//extern const uint16_t AX12A_DEFAULT_PUNCH;

// Value limit definitions
// ----------------------------------------------------------------------------
//extern const uint8_t AX12A_MAX_VELOCITY;




// Types & enums
// ----------------------------------------------------------------------------




// Classes and structs
// ----------------------------------------------------------------------------
class AX12A : Motor{
public:
    /** @see Motor */
    AX12A(
        uint8_t id,
        DaisyChain* daisyChain,
        ResolutionDivider divider
    );

    ~AX12A();


    // Setters (use the WRITE DATA instruction)
    // ------------------------------------------------------------------------
    // RAM
    bool setBaudRate(uint32_t baud) const override;
    bool setGoalVelocity(float goalVelocity) const override;

    // TODO: Consider making these use enums
    void setCWComplianceMargin(uint8_t CWcomplianceMargin);
    void setCCWComplianceMargin(uint8_t CCWcomplianceMargin);
    void setCWComplianceSlope(uint8_t CWcomplianceSlope);
    void setCCWComplianceSlope(uint8_t CCWcomplianceSlope);

    void setComplianceSlope(uint8_t complianceSlope);
    void setComplianceMargin(uint8_t complianceSlope);


    // Getters (use the READ DATA instruction)
    // ------------------------------------------------------------------------
    bool getVelocity(float& retVal) const override;
};

} // end namespace dynamixel




/**
 * @}
 */
/* end - AX12A */

#endif /* AX12A_H */
