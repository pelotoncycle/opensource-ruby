/********************************************************************************************
 *     LEGAL DISCLAIMER
 *
 *     (Header of MediaTek Software/Firmware Release or Documentation)
 *
 *     BY OPENING OR USING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 *     THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE") RECEIVED
 *     FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON AN "AS-IS" BASIS
 *     ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES, EXPRESS OR IMPLIED,
 *     INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
 *     A PARTICULAR PURPOSE OR NONINFRINGEMENT. NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY
 *     WHATSOEVER WITH RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY,
 *     INCORPORATED IN, OR SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK
 *     ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
 *     NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S SPECIFICATION
 *     OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
 *
 *     BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE LIABILITY WITH
 *     RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE, AT MEDIATEK'S OPTION,
 *     TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE, OR REFUND ANY SOFTWARE LICENSE
 *     FEES OR SERVICE CHARGE PAID BY BUYER TO MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 *     THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE WITH THE LAWS
 *     OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF LAWS PRINCIPLES.
 ************************************************************************************************/
/*-----------------------------------------------------------------------------
 *
 * Description:
 *
 *---------------------------------------------------------------------------*/

#ifndef X_TYPEDEF_H
#define X_TYPEDEF_H

/*-----------------------------------------------------------------------------
                    macros, defines, typedefs, enums
 ----------------------------------------------------------------------------*/
#ifndef VOID
#define VOID void
#endif

#if !defined (_NO_TYPEDEF_BYTE_) && !defined (_TYPEDEF_BYTE_)
typedef unsigned char  BYTE;
#define _TYPEDEF_BYTE_
#endif

#if !defined (_NO_TYPEDEF_UCHAR_) && !defined (_TYPEDEF_UCHAR_)
typedef unsigned char  UCHAR;
#define _TYPEDEF_UCHAR_
#endif

#if !defined (_NO_TYPEDEF_UINT8_) && !defined (_TYPEDEF_UINT8_)
typedef unsigned char  UINT8;
#define _TYPEDEF_UINT8_
#endif

#if !defined (_NO_TYPEDEF_UINT16_) && !defined (_TYPEDEF_UINT16_)
typedef unsigned short  UINT16;
#define _TYPEDEF_UINT16_
#endif

#if !defined (_NO_TYPEDEF_UINT32_) && !defined (_TYPEDEF_UINT32_)
typedef unsigned int  UINT32;
#define _TYPEDEF_UINT32_
#endif

#if !defined (_NO_TYPEDEF_UINT64_) && !defined (_TYPEDEF_UINT64_)
typedef unsigned long long  UINT64;
#define _TYPEDEF_UINT64_
#endif

#if !defined (_NO_TYPEDEF_CHAR_) && !defined (_TYPEDEF_CHAR_)
typedef char  CHAR;     // Debug, should be 'signed char'
#define _TYPEDEF_CHAR_
#endif

#if !defined (_NO_TYPEDEF_INT8_) && !defined (_TYPEDEF_INT8_)
typedef signed char  INT8;
#define _TYPEDEF_INT8_
#endif

#if !defined (_NO_TYPEDEF_INT16_) && !defined (_TYPEDEF_INT16_)
typedef signed short  INT16;
#define _TYPEDEF_INT16_
#endif

#if !defined (_NO_TYPEDEF_INT32_) && !defined (_TYPEDEF_INT32_)
typedef signed long  INT32;
#define _TYPEDEF_INT32_
#endif

#if !defined (_NO_TYPEDEF_INT64_) && !defined (_TYPEDEF_INT64_)
typedef signed long long  INT64;
#define _TYPEDEF_INT64_
#endif

/* Define a boolean as 8 bits. */
#if !defined (_NO_TYPEDEF_BOOL_) && !defined (_TYPEDEF_BOOL_)
typedef UINT8  BOOL;
#define _TYPEDEF_BOOL_
#endif

#if !defined (_NO_TYPEDEF_FLOAT_) && !defined (_TYPEDEF_FLOAT_)
typedef float  FLOAT;
#define _TYPEDEF_FLOAT_
#endif

#if !defined (_NO_TYPEDEF_DOUBLE_)  && !defined (_TYPEDEF_DOUBLE_)
typedef double  DOUBLE;
#define _TYPEDEF_DOUBLE_
#endif

#if !defined (_NO_TYPEDEF_DWRD_)  && !defined (_TYPEDEF_DWRD_)
typedef unsigned long DWRD;
#define _TYPEDEF_DWRD_
#endif

#if !defined (_NO_TYPEDEF_WORD_)  && !defined (_TYPEDEF_WORD_)
typedef unsigned short    WORD;
#define _TYPEDEF_WORD_
#endif

#ifndef UNUSED
#define UNUSED(x)               (void)x
#endif

#ifndef MIN
#define MIN(x, y)               (((x) < (y)) ? (x) : (y))
#endif

#ifndef MAX
#define MAX(x, y)               (((x) > (y)) ? (x) : (y))
#endif

#ifndef ABS
#define ABS(x)                  (((x) >= 0) ? (x) : -(x))
#endif

#ifndef DIFF
#define DIFF(x, y)              (((x) > (y)) ? ((x) - (y)) : ((y) - (x)))
#endif

#ifndef NULL
    #define NULL                0
#endif  // NULL

#ifndef TRUE
    #define TRUE                (0 == 0)
#endif  // TRUE

#ifndef FALSE
    #define FALSE               (0 != 0)
#endif  // FALSE


#ifdef __arm
    #define INLINE              __inline
    #define IRQ                 __irq
    #define FIQ                 __irq
#else
    #define INLINE
    #define IRQ
    #define FIQ
#endif // __arm


#ifndef externC
    #ifdef __cplusplus
        #define externC         extern "C"
    #else
        #define externC         extern
    #endif
#endif  // externC


#ifndef EXTERN
    #ifdef __cplusplus
        #define EXTERN          extern "C"
    #else
        #define EXTERN          extern
    #endif
#endif  // EXTERN

#define __align(x)			__attribute__((aligned(x)))

#endif // X_TYPEDEF_H
