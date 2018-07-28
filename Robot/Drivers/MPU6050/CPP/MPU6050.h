/**
  *****************************************************************************
  * @file    example_cpp.h
  * @author  TODO -- your name here
  * @brief   TODO -- briefly describe this file
  *
  * @defgroup TODO -- module name defined in template.c with " (header)" after it
  * @brief    TODO -- brief description of header
  * @ingroup  TODO -- module name defined in template.c
  * @{
  *****************************************************************************
  */




/******************** Define to prevent recursive inclusion ******************/
##ifndef __EXAMPLE_CPP_H__
##define __EXAMPLE_CPP_H__




/********************************* Includes **********************************/
#include <stdint.h>




/********************************** Macros ***********************************/



/********************************** Namespace ********************************/
/*!
 *  \addtogroup Namespace
 *  @{
 */
namespace Namespace {
/********************************* Constants *********************************/




/********************************** Classes **********************************/
//!  A test class.
/*!
  A more elaborate class description.
*/
class MPU6050 {

/****************************** Public Members *******************************/
  public:
    //! A constructor.
    /*!
      The constructor will initialize all aspects of the class, but will not
      perform any I/O on the MPU6050.
    */
    MPU6050();
    //! A destructor.
    /*!
      A more elaborate description of the destructor.
    */


    init();
    int Write_Reg(uint8_t reg_addr, uint8_t data);
    uint8_t Read_Reg(uint8_t reg_addr)
    ~MPU6050();

    //! A normal member taking two arguments and returning an integer value.
    /*!
      \param a an integer argument.
      \param s a constant character pointer.
      \return The test results
    */
    int testMe(int a,const char *s);

/****************************** Protected Members ***************************/
    protected:


/****************************** Private Members *****************************/
    private:

}; //class Class



} //namespace Namespace
/**
 * @}
 */
/**/

#endif /* __EXAMPLE_CPP_H__ */
